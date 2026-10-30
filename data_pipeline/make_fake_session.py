"""Writes a small synthetic Observer session (MujocoFull recorder mode, as LuckyEngine 2026.1.2 writes it) and its sidecar,
to test the scripts without the engine. The recorded ctrl lags the script's command by --lag rows (what the alignment check
finds); the meta ends with the empty episode StopRecording adds.

    python make_fake_session.py --out /tmp/fake --episodes 3 --frames 120 --lag 1
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import av
import numpy as np
import pandas as pd

from common import PHASES, PROBE, SIDECAR_ACTIONS, UPPER_JOINTS, mjcf_ranges

LEGS = [f"{s}_{j}_joint" for s in ("left", "right") for j in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")]
JOINTS = LEGS + UPPER_JOINTS[28:] + UPPER_JOINTS[:28]  # every actuated joint (the engine's order differs; the readers go by name)
FREE = ("x", "y", "z", "qw", "qx", "qy", "qz")


def write_video(path: Path, n: int, h: int, w: int, fps: int, hue: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with av.open(str(path), "w") as c:
        s = c.add_stream("libx264", rate=fps)
        s.width, s.height, s.pix_fmt = w, h, "yuv420p"
        for i in range(n):
            img = np.zeros((h, w, 3), np.uint8)
            img[..., 0], img[..., 1] = hue, (i * 255 // max(n - 1, 1))
            img[..., 2] = (i * 37) % 256  # blue jumps every frame, so a one-frame offset shows
            for p in s.encode(av.VideoFrame.from_ndarray(img, format="rgb24")):
                c.mux(p)
        for p in s.encode():
            c.mux(p)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--episodes", type=int, default=3)
    ap.add_argument("--frames", type=int, default=120)
    ap.add_argument("--lag", type=int, default=1, help="rows between a control tick and the row holding its ctrl")
    ap.add_argument("--fps", type=int, default=50)
    ap.add_argument("--cameras", nargs="+", default=["head_camera", "left_wrist_camera", "right_wrist_camera"])
    ap.add_argument("--size", default="48x64", help="HxW of the fake videos")
    ap.add_argument("--mjcf", help="robot MJCF: keeps the fake commands inside the joint ranges")
    a = ap.parse_args()
    h, w = map(int, a.size.split("x"))
    root, side = Path(a.out) / "session_fake", Path(a.out) / "sidecar_fake"
    rng = np.random.default_rng(0)
    rg = mjcf_ranges(a.mjcf) if a.mjcf else {}
    mid = np.array([sum(rg.get(j, (0, 0))) / 2 for j in UPPER_JOINTS])
    half = np.array([(rg[j][1] - rg[j][0]) / 2 if j in rg else 1.0 for j in UPPER_JOINTS])
    metas, index = [], 0
    for e in range(a.episodes):
        n = a.frames
        t = np.arange(n) / a.fps
        cmd = mid + 0.5 * half * np.sin(t[:, None] * (1 + np.arange(len(UPPER_JOINTS)))[None] * 0.3)  # what the script commands
        ctrl = np.vstack([np.zeros((a.lag, cmd.shape[1])), cmd[: n - a.lag]])  # what the Observer row holds
        qpos = ctrl + rng.normal(0, 0.01, ctrl.shape)
        phase = np.minimum((np.arange(n) * len(PHASES)) // n, len(PHASES) - 1)
        yaw = 0.3 * np.sin(t)  # the pelvis turns, so the derived gyro has something to show
        pelvis = np.column_stack([np.zeros(n), np.zeros(n), np.full(n, 0.56), np.cos(yaw / 2), np.zeros(n), np.zeros(n), np.sin(yaw / 2)])
        box = np.tile([0.6, 0.1, 0.6, 1, 0, 0, 0], (n, 1))
        legs = np.zeros((n, len(LEGS)))  # the walker's joints: recorded as 0 by MujocoFull
        q_all = np.column_stack([legs, qpos[:, 28:], qpos[:, :28]])
        u_all = np.column_stack([legs, ctrl[:, 28:], ctrl[:, :28]])
        rows = {"index": np.arange(index, index + n), "sim_id": np.full(n, 7), "timestamp": t, "episode_index": np.full(n, e),
                "frame_index": np.arange(n), "task_index": phase, "frame_flags": (1 | 2 * (np.arange(n) == 0)).astype(np.uint8),
                "action": list(u_all), "observation.state": list(np.column_stack([pelvis, q_all, box]))}
        (root / "data" / "chunk-000").mkdir(parents=True, exist_ok=True)
        pd.DataFrame(rows).to_parquet(root / "data" / "chunk-000" / f"file-{e:03d}.parquet")
        for k, cam in enumerate(a.cameras):
            write_video(root / "videos" / f"observation.images.{cam}" / "chunk-000" / f"file-{e:03d}.mp4", n, h, w, a.fps, 60 * k)
        metas.append({"episode_index": e, "length": n, "success": e != 1, "data/file_index": e,
                      **{f"videos/observation.images.{c}/{k}": v for c in a.cameras for k, v in (("chunk_index", 0), ("file_index", e), ("from_timestamp", 0.0))}})
        index += n

        # Sidecar: one row per control tick (the command as written), header with the episode's parameters
        side.mkdir(parents=True, exist_ok=True)
        sc = pd.DataFrame({"frame": np.arange(n), "sim_time": t, "task_index": phase})
        sc[SIDECAR_ACTIONS] = np.column_stack([(phase >= 2) & (phase <= 3), 0.4 * (phase == 3), np.zeros(n), 0.2 * (phase == 3)])
        sc["probe_ctrl"] = cmd[:, UPPER_JOINTS.index(PROBE[2:])]
        sc.to_csv(side / f"episode_{e:06d}.csv", index=False)
        (side / f"episode_{e:06d}.json").write_text(json.dumps({
            "episode": e, "seed": 1000 + e, "success": e != 1, "fail_reason": "" if e != 1 else "box slipped out of the hands (0.051 m)",
            "box": {"tag": "PoolBox_01", "width": 0.3, "depth": 0.15, "height": 0.15, "mass": 1.0},
            "box_start": [0.0, 0.01 * e, 0.0], "robot_start": [0.0, 0.0, 0.0], "perturbation": "none",
            "release_error": {"height": 0.012, "planar": 0.02}, "max_slip": 0.01, "max_wrist_dev_deg": 4.0}, indent=1))
    (root / "meta" / "episodes" / "chunk-000").mkdir(parents=True, exist_ok=True)
    metas.append({**metas[-1], "length": 0, "success": False, "data/file_index": a.episodes, **{f"videos/observation.images.{c}/file_index": a.episodes for c in a.cameras}})
    pd.DataFrame(metas).to_parquet(root / "meta" / "episodes" / "chunk-000" / "file-000.parquet")
    names = [f"floating_base_joint_{k}" for k in FREE] + JOINTS + [f"PoolBox_01_{k}" for k in FREE]
    (root / "meta" / "info.json").write_text(json.dumps({"fps": a.fps, "chunks_size": 40, "total_episodes": a.episodes + 1, "features": {
        "frame_flags": {"dtype": "uint8", "shape": [1], "names": ["new_policy_step", "post_reset"]},
        "action": {"dtype": "float64", "shape": [len(JOINTS)], "names": JOINTS},
        "observation.state": {"dtype": "float64", "shape": [len(names)], "names": names}}}))
    print(f"wrote {root} and {side}: {a.episodes} episodes x {a.frames} frames, ctrl lag {a.lag}")


if __name__ == "__main__":
    main()
