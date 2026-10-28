"""Shared by every data_pipeline script: the state and action layouts, the schema column names, the sidecar
format, and readers for an Observer session folder and its sidecar."""

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
IMU = {"imu.torso_gyro": "imu-torso-angular-velocity", "imu.torso_acc": "imu-torso-linear-acceleration"}


def q(joint: str) -> str:
    return f"q.{joint}"


def u(joint: str) -> str:
    return f"u.{joint}"


# Policy vectors: observation.state (37) and action (35)
STATE_COLUMNS = [q(j) for j in UPPER_JOINTS] + list(IMU)  # Observer columns (each IMU column is 3 wide)
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
    ent = next((c for c in ("entity_id", "robot_id", "sim_id") if c in data.columns), None)
    if ent is not None:
        keep = entity if entity is not None else data[ent].mode().iat[0]  # the robot with the most rows
        data = data[data[ent] == keep]
    metas = read_parquets(list(root.glob("meta/episodes/chunk-*/file-*.parquet")))
    meta_by_ep = {int(r["episode_index"]): r for r in metas.to_dict("records")} if not metas.empty else {}
    per_chunk = int(info.get("chunks_size", 40))
    streams = sorted(p.name for p in (root / "videos").glob("*") if p.is_dir())
    episodes = []
    for idx, rows in data.groupby("episode_index"):
        idx = int(idx)
        meta = meta_by_ep.get(idx, {})
        ep = Episode(idx, rows.sort_values("frame_index").reset_index(drop=True), meta=meta)
        for s in streams:
            chunk = meta.get(f"videos/{s}/chunk_index", idx // per_chunk)
            file = meta.get(f"videos/{s}/file_index", idx % per_chunk)
            path = root / "videos" / s / f"chunk-{int(chunk):03d}" / f"file-{int(file):03d}.mp4"
            if path.exists():
                ep.videos[s] = path
                ep.video_start[s] = float(meta.get(f"videos/{s}/from_timestamp", 0.0))
        episodes.append(ep)
    return info, episodes


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


def scene_boxes(scene: str | Path) -> list[tuple[str, str]]:
    """(tag, MuJoCo body name) of every free box in a .hscene: an entity with a MujocoBoxColliderComponent and
    a MujocoJointComponent becomes body lucky_body_<entity id>."""
    out = []
    for block in re.split(r"\n  - Entity: ", Path(scene).read_text())[1:]:
        if "MujocoBoxColliderComponent" in block and "MujocoJointComponent" in block:
            eid = block.split("\n", 1)[0].strip()
            tag = re.search(r"Tag: (.+)", block)
            out.append((tag.group(1).strip() if tag else eid, f"lucky_body_{eid}"))
    return out
