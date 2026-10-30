"""Converts Observer sessions + the script's sidecars into one LeRobot 3.0 dataset for ACT: observation.state (37),
action (35), three camera videos and a phase label, successful episodes only by default.

    python le2act.py --session DataSessions/session_X --sidecar DataSidecars/run_X \
        --cameras head=head_camera wrist_left=left_wrist_camera wrist_right=right_wrist_camera \
        --out datasets/g1_box_act --repo-id local/g1_box_act --size 256x256
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import av
import numpy as np

from common import ACTION_NAMES, ROBOT_TYPE, STATE_NAMES, TASK, action_shift, episode_arrays, read_session, read_sidecar, segment


def video_frames(path: Path, start: float, fps: float, h: int, w: int):
    """RGB frames (h, w, 3) of one episode's video, resized, from its start timestamp on."""
    with av.open(str(path)) as c:
        for f in c.decode(video=0):
            if f.time is None or f.time >= start - 0.5 / fps:
                yield f.to_ndarray(format="rgb24", width=w, height=h)


def count_frames(path: Path) -> int:
    with av.open(str(path)) as c:
        return sum(1 for _ in c.decode(video=0))


def features(cams: dict[str, str], h: int, w: int) -> dict:
    return {
        "observation.state": {"dtype": "float32", "shape": (len(STATE_NAMES),), "names": STATE_NAMES},
        "action": {"dtype": "float32", "shape": (len(ACTION_NAMES),), "names": ACTION_NAMES},
        "phase": {"dtype": "int64", "shape": (1,), "names": None},
        **{f"observation.images.{k}": {"dtype": "video", "shape": (h, w, 3), "names": ["height", "width", "channels"]} for k in cams},
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", nargs="+", required=True, help="Observer session folder(s)")
    ap.add_argument("--sidecar", nargs="+", required=True, help="sidecar folder(s), one per session, same order")
    ap.add_argument("--cameras", nargs="+", required=True, help="key=OutputName, e.g. head=head_camera")
    ap.add_argument("--out", required=True, help="dataset root to create")
    ap.add_argument("--repo-id", default="local/g1_box_act")
    ap.add_argument("--size", default="256x256", help="HxW the frames are resized to")
    ap.add_argument("--shift", default="auto", help="rows from a control tick to its ctrl: auto, 0, 1, ...")
    ap.add_argument("--segment", default="all", choices=["all", "reach", "carry"], help="whole episode or one half")
    ap.add_argument("--include-failures", action="store_true")
    ap.add_argument("--dry-run", action="store_true", help="check and report only, write nothing")
    a = ap.parse_args()
    if len(a.session) != len(a.sidecar):
        ap.error("give one --sidecar per --session")
    cams = dict(c.split("=", 1) for c in a.cameras)
    streams = {k: f"observation.images.{v}" for k, v in cams.items()}
    h, w = map(int, a.size.split("x"))

    ds, fps0, kept, skipped = None, None, [], []
    for sess, sc in zip(a.session, a.sidecar):
        info, episodes = read_session(sess)  # one session in memory at a time
        fps = float(info.get("fps", 50))
        for ep in episodes:
            tag = f"{Path(sess).name} ep {ep.index:4d}"
            side = read_sidecar(sc, ep.index)
            ok = side.header.get("success", ep.meta.get("success", True)) if side else False
            missing = [s for s in streams.values() if s not in ep.videos]
            why = ("no sidecar" if side is None else f"no video for {missing}" if missing
                   else "" if ok or a.include_failures else f"failed: {side.header.get('fail_reason', '')}")
            if not why:
                shift, err = action_shift(ep.rows, side) if a.shift == "auto" else (int(a.shift), 0.0)
                state, action, phase = episode_arrays(ep, side, shift)
                n = min([len(state)] + [count_frames(ep.videos[s]) for s in streams.values()])
                cut = segment(phase[:n], a.segment)
                why = "" if cut.stop > cut.start else f"empty '{a.segment}' segment"
            if why:
                print(f"{tag}: skip ({why})")
                skipped.append({"session": sess, "episode": ep.index, "why": why})
                continue
            print(f"{tag}: {cut.stop - cut.start} frames, action shift {shift} (probe error {err:.2e})")
            kept.append({"session": sess, "source_episode": ep.index, "frames": cut.stop - cut.start, "shift": shift,
                         "seed": side.header.get("seed"), "box": side.header.get("box"), "success": bool(ok)})
            if a.dry_run:
                continue

            # Write through LeRobot so the format is exactly what lerobot-train reads
            if ds is None:
                from lerobot.datasets.lerobot_dataset import LeRobotDataset

                fps0 = fps
                ds = LeRobotDataset.create(a.repo_id, int(fps), features(cams, h, w), root=a.out, robot_type=ROBOT_TYPE, use_videos=True)
            if fps != fps0:
                print(f"{sess} records at {fps} Hz, the dataset at {fps0} Hz", file=sys.stderr)
                return 1
            gens = {k: video_frames(ep.videos[s], ep.video_start[s] + cut.start / fps, fps, h, w) for k, s in streams.items()}
            for t in range(cut.start, cut.stop):
                frame = {"observation.state": state[t].astype(np.float32), "action": action[t].astype(np.float32),
                         "phase": np.array([phase[t]], np.int64), "task": TASK}
                frame.update({f"observation.images.{k}": next(g) for k, g in gens.items()})
                ds.add_frame(frame)
            ds.save_episode()
    print(f"{len(kept)} of {len(kept) + len(skipped)} episodes kept, {sum(k['frames'] for k in kept)} frames")
    if ds is not None:
        ds.finalize()
        (Path(a.out) / "conversion_report.json").write_text(json.dumps({"kept": kept, "skipped": skipped}, indent=1))
        print(f"wrote {a.out}")
    return 0 if kept else 1


if __name__ == "__main__":
    sys.exit(main())
