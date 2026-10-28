"""Checks an Observer session and its sidecar before conversion, and summarises every episode. Hard errors (missing
or NaN data, frame count mismatches, resets inside an episode, actions out of range, unclear alignment) exit 1.

    python qa.py --session DataSessions/session_X --sidecar DataSidecars/run_X --mjcf .../g1_three_fingers.xml \
        --cameras HeadCam LeftWristCam RightWristCam --csv qa_session_X.csv
"""

from __future__ import annotations

import argparse
import sys
from collections import Counter

import numpy as np
import pandas as pd

from common import PHASES, POST_RESET, STATE_COLUMNS, UPPER_JOINTS, action_shift, matrix, mjcf_ranges, read_session, read_sidecar, u
from le2act import count_frames


def histogram(values: list[float], bins: int = 8, width: int = 30) -> list[str]:
    v = np.asarray([x for x in values if x is not None], float)
    if len(v) == 0:
        return ["   (none)"]
    counts, edges = np.histogram(v, bins=bins)
    return [f"   {edges[i]:8.3f} .. {edges[i + 1]:8.3f} {'#' * int(width * c / max(counts.max(), 1))} {c}" for i, c in enumerate(counts)]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", required=True)
    ap.add_argument("--sidecar", required=True)
    ap.add_argument("--mjcf", required=True, help="robot MJCF, for the actuator ranges")
    ap.add_argument("--cameras", nargs="+", required=True, help="camera Output Names")
    ap.add_argument("--csv", help="write the per-episode summary here")
    ap.add_argument("--max-shift-err", type=float, default=1e-4, help="probe mismatch that makes the alignment unclear")
    a = ap.parse_args()

    info, episodes = read_session(a.session)
    fps = float(info.get("fps", 50))
    ranges = mjcf_ranges(a.mjcf)
    lo = np.array([ranges.get(j, (-np.inf, np.inf))[0] for j in UPPER_JOINTS]) - 1e-6
    hi = np.array([ranges.get(j, (-np.inf, np.inf))[1] for j in UPPER_JOINTS]) + 1e-6
    errors, rows, shifts = [], [], Counter()
    for ep in episodes:
        e, r = ep.index, ep.rows
        side = read_sidecar(a.sidecar, e)
        bad = lambda msg: errors.append(f"episode {e}: {msg}")  # noqa: E731
        state, ctrl = matrix(r, STATE_COLUMNS), matrix(r, [u(j) for j in UPPER_JOINTS])

        # Integrity: values, frame counts, resets
        if np.isnan(state).any() or np.isnan(ctrl).any():
            bad("NaN in state or ctrl")
        if not np.array_equal(r["frame_index"].to_numpy(), np.arange(len(r))):
            bad("frame_index has gaps")
        for cam in a.cameras:
            s = f"observation.images.{cam}"
            if s not in ep.videos:
                bad(f"no video {s}")
            elif (n := count_frames(ep.videos[s])) != len(r):
                bad(f"{cam}: {n} video frames for {len(r)} rows")
        if POST_RESET in r and (resets := np.flatnonzero(r[POST_RESET].to_numpy())).size and (resets != 0).any():
            bad(f"reset inside the episode at frames {resets[resets != 0][:5].tolist()}")
        out = ((ctrl < lo) | (ctrl > hi)).any(axis=0)
        if out.any():
            bad(f"ctrl outside the joint range: {[UPPER_JOINTS[i] for i in np.flatnonzero(out)][:5]}")

        # Sidecar: present, same length, alignment found without doubt
        row = {"episode": e, "frames": len(r), "seconds": len(r) / fps}
        if side is None:
            bad("no sidecar")
        else:
            if abs(len(side.rows) - len(r)) > 2:
                bad(f"sidecar has {len(side.rows)} ticks for {len(r)} rows")
            shift, err = action_shift(r, side)
            shifts[shift] += 1
            if err > a.max_shift_err:
                bad(f"alignment unclear: best shift {shift} still differs by {err:.2e}")
            if not set(np.unique(side.rows["waist_lock"])) <= {0, 1}:
                bad("waist_lock not 0/1")
            hd = side.header
            phase = side.rows["task_index"].to_numpy()
            row.update({"success": hd.get("success"), "fail_reason": hd.get("fail_reason", ""), "shift": shift, "seed": hd.get("seed"),
                        "box_width": (hd.get("box") or {}).get("width"), "box_mass": (hd.get("box") or {}).get("mass"),
                        "perturbation": hd.get("perturbation"), "max_slip": hd.get("max_slip"), "max_wrist_dev_deg": hd.get("max_wrist_dev_deg"),
                        "release_height": (hd.get("release_error") or {}).get("height"), "release_planar": (hd.get("release_error") or {}).get("planar"),
                        **{f"t_{p}": float((phase == i).sum() / fps) for i, p in enumerate(PHASES)}})
        rows.append(row)

    # Report
    df = pd.DataFrame(rows)
    pd.set_option("display.width", 200)
    cols = [c for c in ["episode", "frames", "success", "shift", "box_width", "box_mass", "perturbation", "max_slip", "release_height", "fail_reason"] if c in df]
    print(df[cols].to_string(index=False))
    if "success" in df:
        print(f"\nsuccess {int(df['success'].fillna(False).sum())} / {len(df)}, action shifts {dict(shifts)}, "
              f"mean episode {df['seconds'].mean():.1f} s")
        print("time per phase (mean s): " + ", ".join(f"{p} {df[f't_{p}'].mean():.1f}" for p in PHASES if f"t_{p}" in df))
        for k in ("box_width", "box_mass"):
            print(f"{k}:\n" + "\n".join(histogram(df[k].tolist())))
    if a.csv:
        df.to_csv(a.csv, index=False)
    if len(shifts) > 1:
        errors.append(f"episodes disagree on the action shift: {dict(shifts)}")
    print(f"\n{len(errors)} error(s)" + "".join(f"\n  {m}" for m in errors[:50]))
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
