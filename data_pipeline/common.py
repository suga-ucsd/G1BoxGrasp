"""Shared by every data_pipeline script: the state and action layouts, the column names, the sidecar format, and
readers for an Observer session folder (MujocoFull recorder mode) and its sidecar."""

from __future__ import annotations

import json
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd

# Joints the policy sees and drives (actuators are named after their joints in g1_three_fingers.xml)
SIDES = ("left", "right")
ARM = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
HAND = ("thumb_0", "thumb_1", "thumb_2", "index_0", "index_1", "middle_0", "middle_1")
ARM_JOINTS = [f"{s}_{j}_joint" for s in SIDES for j in ARM]
HAND_JOINTS = [f"{s}_hand_{j}_joint" for s in SIDES for j in HAND]
WAIST_JOINTS = ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
UPPER_JOINTS = ARM_JOINTS + HAND_JOINTS + WAIST_JOINTS  # 31
IMU = ["imu.pelvis_gravity", "imu.pelvis_gyro"]  # from the pelvis orientation: gravity in the pelvis frame, angular velocity (rad/s)


def q(joint: str) -> str:
    return f"q.{joint}"


def u(joint: str) -> str:
    return f"u.{joint}"


# Policy vectors: observation.state (37) and action (35)
STATE_COLUMNS = [q(j) for j in UPPER_JOINTS] + IMU  # session columns after expand_mujoco_full (each IMU column is 3 wide)
STATE_NAMES = [q(j) for j in UPPER_JOINTS] + [f"{k}.{a}" for k in IMU for a in "xyz"]
SIDECAR_ACTIONS = ["waist_lock", "cmd_vx", "cmd_vy", "cmd_yaw"]
ACTION_NAMES = [u(j) for j in UPPER_JOINTS] + SIDECAR_ACTIONS
STATUS_NAMES = ["status.done", "status.success", "status.phase"]  # appended to the state by the eval agent

# Episode labels
PHASES = ["catch", "grasp", "lift", "carry", "place", "retract"]  # task_index = position
TASK = "Catch the box sliding down the conveyor and place it on the table"
PROBE = u("left_wrist_roll_joint")  # script-written ctrl, echoed in the sidecar as probe_ctrl
POST_RESET = "flag.post_reset"
ROBOT_TYPE = "unitree_g1_three_fingers"


@dataclass
class Episode:
    """One Observer episode: its data rows (one robot, sorted by frame) and its video file per stream."""

    index: int
    rows: pd.DataFrame
    videos: dict[str, Path] = field(default_factory=dict)
    video_start: dict[str, float] = field(default_factory=dict)
    meta: dict = field(default_factory=dict)


@dataclass
class Sidecar:
    """The script's file pair for one episode: header (seed, parameters, outcome) and one row per control tick."""

    header: dict
    rows: pd.DataFrame


def read_parquets(paths: list[Path]) -> pd.DataFrame:
    frames = [pd.read_parquet(p) for p in sorted(paths)]
    return pd.concat(frames, ignore_index=True) if frames else pd.DataFrame()


def read_session(root: str | Path, entity: int | None = None) -> tuple[dict, list[Episode]]:
    """Episodes of an Observer session folder, with video files found from meta/episodes (or the documented
    chunk rule: episode N in chunk N // files_per_chunk, file N % files_per_chunk)."""
    root = Path(root)
    info = json.loads((root / "meta" / "info.json").read_text()) if (root / "meta" / "info.json").exists() else {}
    data = read_parquets(list(root.glob("data/chunk-*/file-*.parquet")))
    if data.empty:
        raise FileNotFoundError(f"no data parquet files under {root}/data")
    data = expand_mujoco_full(data, info)
    ent = next((c for c in ("entity_id", "robot_id", "sim_id") if c in data.columns), None)
    if ent is not None:
        keep = entity if entity is not None else data[ent].mode().iat[0]  # the robot with the most rows
        data = data[data[ent] == keep]
    metas = read_parquets(list(root.glob("meta/episodes/chunk-*/file-*.parquet")))
    meta_by_ep = {int(r["episode_index"]): r for r in metas.to_dict("records") if r.get("length", 1)} if not metas.empty else {}  # StopRecording adds an empty one
    per_chunk = int(info.get("chunks_size", 40))
    streams = sorted(p.name for p in (root / "videos").glob("*") if p.is_dir())
    episodes = []
    for idx, rows in data.groupby("episode_index"):
        idx = int(idx)
        meta = meta_by_ep.get(idx, {})
        rows = rows.sort_values("frame_index").reset_index(drop=True)
        rows[IMU[0]], rows[IMU[1]] = pelvis_imu(np.stack(rows["pose.pelvis"].to_numpy()), float(info.get("fps", 50)))
        ep = Episode(idx, rows, meta=meta)
        for s in streams:
            chunk = meta.get(f"videos/{s}/chunk_index", idx // per_chunk)
            file = meta.get(f"videos/{s}/file_index", idx % per_chunk)
            path = root / "videos" / s / f"chunk-{int(chunk):03d}" / f"file-{int(file):03d}.mp4"
            if path.exists():
                ep.videos[s] = path
                ep.video_start[s] = float(meta.get(f"videos/{s}/from_timestamp", 0.0))
        episodes.append(ep)
    return info, episodes


def expand_mujoco_full(data: pd.DataFrame, info: dict) -> pd.DataFrame:
    """MujocoFull rows -> named columns: q.<joint> from observation.state (the whole qpos), u.<joint> from action (the
    actuator commands), pose.pelvis and pose.box.<Tag> (x, y, z, qw, qx, qy, qz; MuJoCo frame) from the free joints, and
    the post-reset bit of frame_flags."""
    feats = info["features"]
    state, action = np.stack(data["observation.state"].to_numpy()), np.stack(data["action"].to_numpy())
    cols = {q(n): state[:, i] for i, n in enumerate(feats["observation.state"]["names"]) if not n.endswith(("_x", "_y", "_z", "_qw", "_qx", "_qy", "_qz"))}
    cols |= {u(n): action[:, i] for i, n in enumerate(feats["action"]["names"])}
    names = feats["observation.state"]["names"]
    for i in [k for k, n in enumerate(names) if n.endswith("_x") and names[k + 6 : k + 7] == [n[:-2] + "_qz"]]:
        body = names[i][:-2]
        cols["pose.pelvis" if body == "floating_base_joint" else f"pose.box.{body}"] = list(state[:, i : i + 7])
    cols[POST_RESET] = (data["frame_flags"].to_numpy() >> feats["frame_flags"]["names"].index("post_reset")) & 1
    return pd.concat([data.drop(columns=["observation.state", "action"]).reset_index(drop=True), pd.DataFrame(cols)], axis=1)


def pelvis_imu(pose: np.ndarray, fps: float) -> tuple[list, list]:
    """What the pelvis IMU would give, from the pelvis pose rows (x, y, z, qw, qx, qy, qz): gravity direction in the pelvis
    frame, and angular velocity in the pelvis frame from consecutive orientations (0 on the first row)."""
    w, x, y, z = pose[:, 3:7].T
    grav = np.stack([2 * (w * y - x * z), -2 * (w * x + y * z), -(w * w - x * x - y * y + z * z)], axis=1)  # R^T (0, 0, -1)
    qa, qb = pose[:-1, 3:7], pose[1:, 3:7]
    dq = np.stack([qa[:, 0] * qb[:, 1] - qa[:, 1] * qb[:, 0] - qa[:, 2] * qb[:, 3] + qa[:, 3] * qb[:, 2],
                   qa[:, 0] * qb[:, 2] + qa[:, 1] * qb[:, 3] - qa[:, 2] * qb[:, 0] - qa[:, 3] * qb[:, 1],
                   qa[:, 0] * qb[:, 3] - qa[:, 1] * qb[:, 2] + qa[:, 2] * qb[:, 1] - qa[:, 3] * qb[:, 0]], axis=1)  # vector part of conj(qa) qb
    sign = np.sign(np.sum(qa * qb, axis=1, keepdims=True) + 1e-12)
    gyro = np.vstack([np.zeros((1, 3)), 2 * fps * sign * dq])
    return list(grav), list(gyro)


def read_sidecar(root: str | Path, index: int) -> Sidecar | None:
    """episode_NNNNNN.json + .csv written by the episode runner (None if missing)."""
    base = Path(root) / f"episode_{index:06d}"
    if not base.with_suffix(".csv").exists():
        return None
    header = json.loads(base.with_suffix(".json").read_text()) if base.with_suffix(".json").exists() else {}
    return Sidecar(header, pd.read_csv(base.with_suffix(".csv")).sort_values("frame").reset_index(drop=True))


def matrix(rows: pd.DataFrame, columns: list[str]) -> np.ndarray:
    """(T, n) float array from scalar columns and list-valued columns (a 3-wide IMU column adds 3)."""
    parts = []
    for c in columns:
        v = rows[c].to_numpy()
        parts.append(np.stack(v).astype(np.float64) if v.dtype == object else v.astype(np.float64)[:, None])
    return np.concatenate(parts, axis=1)


def action_shift(rows: pd.DataFrame, side: Sidecar, max_shift: int = 2) -> tuple[int, float]:
    """Rows between a control tick and the Observer row that holds its ctrl: compares the probe actuator's
    recorded ctrl with what the script wrote (sidecar probe_ctrl). Returns (shift, mean abs error)."""
    rec = rows[PROBE].to_numpy(np.float64)
    ref = side.rows["probe_ctrl"].to_numpy(np.float64)
    best = (0, np.inf)
    for s in range(max_shift + 1):
        n = min(len(rec) - s, len(ref))
        if n > 0:
            err = float(np.mean(np.abs(rec[s : s + n] - ref[:n])))
            best = min(best, (s, err), key=lambda b: b[1])
    return best


def episode_arrays(ep: Episode, side: Sidecar, shift: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """state (T, 37), action (T, 35) and phase (T,) of one episode: state at row k, upper-body ctrl at row
    k + shift, walker command and waist lock from sidecar tick k."""
    n = min(len(ep.rows) - shift, len(side.rows))
    state = matrix(ep.rows.iloc[:n], STATE_COLUMNS)
    ctrl = matrix(ep.rows.iloc[shift : shift + n], [u(j) for j in UPPER_JOINTS])
    extra = side.rows[SIDECAR_ACTIONS].to_numpy(np.float64)[:n]
    phase = side.rows["task_index"].to_numpy(np.int64)[:n]
    return state, np.concatenate([ctrl, extra], axis=1), phase


def segment(phase: np.ndarray, which: str) -> slice:
    """Frames of one half for the two-policy option: 'reach' = catch..lift, 'carry' = carry..retract."""
    first_carry = int(np.argmax(phase >= PHASES.index("carry"))) if (phase >= PHASES.index("carry")).any() else len(phase)
    return {"all": slice(0, len(phase)), "reach": slice(0, first_carry), "carry": slice(first_carry, len(phase))}[which]


def mjcf_ranges(path: str | Path) -> dict[str, tuple[float, float]]:
    """Joint ranges from the MJCF (the position actuators inherit them as their ctrl ranges)."""
    ranges = {}
    for j in ET.parse(path).getroot().iter("joint"):
        if j.get("name") and j.get("range"):
            lo, hi = map(float, j.get("range").split())
            ranges[j.get("name")] = (lo, hi)
    return ranges
