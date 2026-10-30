"""Renders one recorded session and its sidecar as a single HTML page, to see whether the data is good before any
training: per-episode checks (pass / warn / fail), the phase timeline, camera frames at every phase start, every
joint's measured angle against its command, the walker command and waist lock, and the box and pelvis paths.

    python inspect_session.py --session DataSessions/session_X --sidecar DataSidecars/run_X \
        --mjcf .../g1_three_fingers.xml --cameras head_camera --out inspect_session_X.html
"""

from __future__ import annotations

import argparse
import base64
import html
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

from common import IMU, PHASES, POST_RESET, PROBE, STATE_COLUMNS, UPPER_JOINTS, Episode, Sidecar, action_shift, matrix, mjcf_ranges, q, read_session, read_sidecar, u
from le2act import video_frames

THUMB = (120, 160)  # h, w of the decoded frames (thumbnails and frozen-frame check: a live render never changes by < 0.02 per pixel)
POINTS = 500  # samples per chart line
GROUPS = [("Left arm", UPPER_JOINTS[0:7]), ("Right arm", UPPER_JOINTS[7:14]), ("Left hand", UPPER_JOINTS[14:21]),
          ("Right hand", UPPER_JOINTS[21:28]), ("Waist", UPPER_JOINTS[28:31])]


def short(joint: str) -> str:
    return joint.removesuffix("_joint").replace("left_", "").replace("right_", "").replace("hand_", "")


def esc(s) -> str:
    return html.escape(str(s))


# ---------- checks ----------

def check_episode(ep: Episode, side: Sidecar | None, fps: float, lo: np.ndarray, hi: np.ndarray, frames: dict[str, np.ndarray]) -> tuple[list, dict]:
    """(status, check, detail) rows for one episode, plus values the charts reuse."""
    r, out = ep.rows, []
    add = lambda status, name, detail: out.append((status, name, detail))  # noqa: E731
    state, ctrl = matrix(r, STATE_COLUMNS), matrix(r, [u(j) for j in UPPER_JOINTS])
    n = len(r)

    # Rows: values, frame counter, tick spacing, resets
    nan = int(np.isnan(state).sum() + np.isnan(ctrl).sum())
    add("fail" if nan else "pass", "No missing values", f"{nan} NaN values in {n} rows" if nan else f"{n} rows, state and ctrl complete")
    gaps = np.flatnonzero(np.diff(r["frame_index"].to_numpy()) != 1)
    add("fail" if gaps.size else "pass", "Frame counter continuous", f"jumps after frames {gaps[:5].tolist()}" if gaps.size else "0, 1, 2, ... without gaps")
    dt = np.diff(r["timestamp"].to_numpy(np.float64))
    if dt.size:
        late = int((dt > 1.5 / fps).sum())
        add("warn" if late else "pass", "Even tick spacing",
            f"median {1000 * np.median(dt):.1f} ms (expected {1000 / fps:.1f}), {late} gaps over {1500 / fps:.0f} ms, longest {1000 * dt.max():.0f} ms")
    if POST_RESET in r:
        resets = np.flatnonzero(r[POST_RESET].to_numpy())
        inside = resets[resets != 0]
        add("fail" if inside.size else "pass", "No reset inside the episode",
            f"post-reset flag at frames {inside[:5].tolist()}" if inside.size else f"post-reset flag {'on frame 0 only' if resets.size else 'never set'}")
    out_of = [UPPER_JOINTS[i] for i in np.flatnonzero(((ctrl < lo) | (ctrl > hi)).any(axis=0))]
    add("fail" if out_of else "pass", "Commands inside the joint ranges", ", ".join(map(short, out_of[:6])) if out_of else "all 31 actuators")

    # Cameras: frame count, frozen runs, blank frames
    for cam, f in frames.items():
        if f is None:
            add("fail", f"{cam}: video present", "no video file for this episode")
            continue
        if len(f) != n:
            add("fail", f"{cam}: one frame per row", f"{len(f)} frames for {n} rows")
        else:
            add("pass", f"{cam}: one frame per row", f"{len(f)} frames")
        same = np.abs(np.diff(f.astype(np.int16), axis=0)).mean(axis=(1, 2, 3)) < 0.02 if len(f) > 1 else np.zeros(0, bool)
        run = longest_run(same)
        add("warn" if run >= fps else "pass", f"{cam}: image keeps changing",
            f"longest run of identical frames {run + 1 if run else 0} ({(run + 1) / fps:.1f} s)" if run else "no two consecutive frames identical")
        dark = int((f.mean(axis=(1, 2, 3)) < 8).sum())
        add("warn" if dark else "pass", f"{cam}: no blank frames", f"{dark} near-black frames" if dark else "none")

    # Sidecar: ticks, alignment, how well the alignment is pinned down
    info = {"shift": 0, "phase": None}
    if side is None:
        add("fail", "Sidecar present", "no episode file in the sidecar folder: no walker command, waist lock or phase")
        return out, info
    m = len(side.rows)
    add("fail" if abs(m - n) > 2 else "pass", "Sidecar ticks match the rows", f"{m} ticks for {n} rows")
    shift, err = action_shift(r, side)
    errs = []
    for s in range(3):
        k = min(n - s, m)
        errs.append(float(np.mean(np.abs(r[PROBE].to_numpy(np.float64)[s:s + k] - side.rows["probe_ctrl"].to_numpy(np.float64)[:k]))) if k > 0 else np.inf)
    spread = float(np.std(side.rows["probe_ctrl"]))
    second = sorted(errs)[1]
    pinned = spread > 1e-3 and second > 10 * max(err, 1e-9)
    add("fail" if err > 1e-4 else "pass" if pinned else "warn", "Actions line up with the script",
        f"recorded ctrl matches the script's command {shift} row(s) later (mean error {err:.1e} rad; shifts 0/1/2: "
        + ", ".join(f"{e:.1e}" for e in errs) + ")" + ("" if pinned else f"; the probe joint barely moves (std {spread:.4f} rad), so the shift isn't pinned down"))
    lock = set(np.unique(side.rows["waist_lock"]).tolist())
    add("pass" if lock <= {0, 1} else "fail", "Waist lock is 0/1", f"values {sorted(lock)}")
    phase = side.rows["task_index"].to_numpy(np.int64)
    seen = [PHASES[i] for i in sorted(set(phase.tolist())) if 0 <= i < len(PHASES)]
    ok = bool(side.header.get("success"))
    add("pass" if not ok or len(seen) == len(PHASES) else "warn", "All phases present", ", ".join(seen))
    back = int((np.diff(phase) < 0).sum())
    add("warn" if back else "pass", "Phases only move forward", f"{back} steps back" if back else "in order")
    add("pass" if ok else "warn", "Expert succeeded", "placed on the table" if ok else side.header.get("fail_reason") or "failed")
    info.update(shift=shift, phase=phase)
    return out, info


def longest_run(flags: np.ndarray) -> int:
    best = cur = 0
    for v in flags:
        cur = cur + 1 if v else 0
        best = max(best, cur)
    return best


# ---------- charts ----------

def chart(t: np.ndarray, series: list[tuple[str, np.ndarray, int]], bounds: list[float], title: str, unit: str, height: int = 120,
          steps: bool = False, legend: bool = False) -> str:
    """Line chart (SVG, hover data attached): series = (name, values, colour slot); bounds = phase start times; legend = the
    page already names the series (else the caption does)."""
    w, h, l, rgt, top, bot = 320, height, 40, 8, 6, 18
    idx = np.unique(np.linspace(0, len(t) - 1, min(len(t), POINTS)).astype(int))
    tt = t[idx]
    vals = [np.asarray(v, np.float64)[idx] for _, v, _ in series]
    allv = np.concatenate([v[np.isfinite(v)] for v in vals]) if vals else np.zeros(1)
    y0, y1 = (float(allv.min()), float(allv.max())) if allv.size else (0.0, 1.0)
    if y1 - y0 < 1e-6:
        y0, y1 = y0 - 0.5, y1 + 0.5
    pad = 0.06 * (y1 - y0)
    y0, y1 = y0 - pad, y1 + pad
    t0, t1 = float(t[0]), float(max(t[-1], t[0] + 1e-6))
    X = lambda x: l + (x - t0) / (t1 - t0) * (w - l - rgt)  # noqa: E731
    Y = lambda y: top + (y1 - y) / (y1 - y0) * (h - top - bot)  # noqa: E731
    parts = [f'<line class="grid" x1="{l}" x2="{w - rgt}" y1="{Y(v):.1f}" y2="{Y(v):.1f}"/>'
             f'<text class="tick" x="{l - 4}" y="{Y(v) + 3:.1f}" text-anchor="end">{v:.3g}</text>' for v in (y0 + pad, y1 - pad)]
    parts += [f'<line class="phase" x1="{X(b):.1f}" x2="{X(b):.1f}" y1="{top}" y2="{h - bot}"/>' for b in bounds[1:]]
    parts += [f'<text class="tick" x="{l}" y="{h - 4}">{t0:.0f} s</text><text class="tick" x="{w - rgt}" y="{h - 4}" text-anchor="end">{t1:.0f} s</text>']
    for (name, _, slot), v in zip(series, vals):
        pts = []
        for i, (x, y) in enumerate(zip(tt, v)):
            if not np.isfinite(y):
                continue  # rows past the end of the shifted command
            if steps and pts:
                pts.append(f"{X(x):.1f},{pts[-1].split(',')[1]}")
            pts.append(f"{X(x):.1f},{Y(y):.1f}")
        parts.append(f'<polyline class="s{slot}" points="{" ".join(pts)}"/>')
    data = {"x": [round(float(x), 3) for x in tt], "s": [[n, sl, [round(float(y), 4) for y in v]] for (n, _, sl), v in zip(series, vals)],
            "u": unit, "box": [l, w - rgt, top, h - bot], "t": [t0, t1]}
    keys = "".join(f'<span class="key s{sl}"></span>{esc(n)}' for n, _, sl in series) if len(series) > 1 and not legend else ""
    return (f'<figure class="chart"><figcaption>{esc(title)}{keys}</figcaption><svg viewBox="0 0 {w} {h}" data-c=\'{esc(json.dumps(data))}\' '
            f'role="img" aria-label="{esc(title)}">{"".join(parts)}<line class="cross" y1="{top}" y2="{h - bot}"/></svg></figure>')


def timeline(phase: np.ndarray | None, fps: float, n: int) -> tuple[str, list[float]]:
    """Phase bar (labelled segments) and the phase start times."""
    if phase is None or not len(phase):
        return "<p class='muted'>No phase labels (no sidecar).</p>", [0.0]
    starts = [0] + (np.flatnonzero(np.diff(phase)) + 1).tolist()
    segs, bounds = [], []
    for i, s in enumerate(starts):
        e = starts[i + 1] if i + 1 < len(starts) else len(phase)
        name = PHASES[phase[s]] if 0 <= phase[s] < len(PHASES) else str(phase[s])
        segs.append(f'<div class="seg p{phase[s] % 2}" style="flex:{e - s}" title="{name}: {s / fps:.1f}-{e / fps:.1f} s">'
                    f'<span>{name} {(e - s) / fps:.1f} s</span></div>')
        bounds.append(s / fps)
    return f'<div class="timeline">{"".join(segs)}</div>', bounds


def thumbs(frames: dict[str, np.ndarray], phase: np.ndarray | None, fps: float) -> str:
    """Each camera's frame at the start of every phase and at the end."""
    if not frames:
        return ""
    picks = [(0, PHASES[phase[0]] if phase is not None and len(phase) else "start")]
    if phase is not None:
        picks += [(int(i) + 1, PHASES[phase[i + 1]]) for i in np.flatnonzero(np.diff(phase))]
    rows = []
    for cam, f in frames.items():
        if f is None or not len(f):
            continue
        cells = []
        for k, label in picks + [(len(f) - 1, "end")]:
            k = min(k, len(f) - 1)
            buf = io.BytesIO()
            Image.fromarray(f[k]).save(buf, "JPEG", quality=80)
            cells.append(f'<figure class="thumb"><img alt="{esc(cam)} at {label}" src="data:image/jpeg;base64,{base64.b64encode(buf.getvalue()).decode()}">'
                         f'<figcaption>{esc(label)} · {k / fps:.1f} s</figcaption></figure>')
        rows.append(f'<h4>{esc(cam)}</h4><div class="thumbs">{"".join(cells)}</div>')
    return "".join(rows)


def episode_html(ep: Episode, side: Sidecar | None, checks: list, info: dict, fps: float, frames: dict, is_open: bool) -> str:
    r, n = ep.rows, len(ep.rows)
    t = np.arange(n) / fps
    bar, bounds = timeline(info["phase"], fps, n)
    fails = sum(c[0] == "fail" for c in checks)
    warns = sum(c[0] == "warn" for c in checks)
    verdict = "fail" if fails else "warn" if warns else "pass"
    ok = side is not None and side.header.get("success")
    head = (f'<summary><span class="badge {verdict}">{ICON[verdict]} {LABEL[verdict]}</span> Episode {ep.index} · {n / fps:.1f} s · '
            f'{"expert succeeded" if ok else "expert failed" if side is not None else "no sidecar"}'
            f'{f" · {fails} failed, {warns} warnings" if fails or warns else ""}</summary>')
    rows = "".join(f'<tr><td><span class="badge {s}">{ICON[s]} {LABEL[s]}</span></td><td>{esc(c)}</td><td>{esc(d)}</td></tr>' for s, c, d in checks)
    body = [bar, f'<table class="checks"><thead><tr><th>Result</th><th>Check</th><th>Detail</th></tr></thead><tbody>{rows}</tbody></table>']
    if side is not None:
        hd = side.header
        box = hd.get("box") or {}
        body.append(f'<p class="muted">Box {box.get("width", "?")} × {box.get("depth", "?")} × {box.get("height", "?")} m, {box.get("mass", "?")} kg; '
                    f'placed {hd.get("release_error", {}).get("planar", "?")} m from its spot; seed {hd.get("seed")}.</p>')
    body.append("<h3>Cameras</h3>" + (thumbs(frames, info["phase"], fps) or "<p class='muted'>No camera streams.</p>"))

    # Joints: measured angle against the command that produced it (ctrl shifted back by the found shift)
    s = info["shift"]
    body.append('<h3>Joints: measured vs commanded</h3><p class="legend"><span class="key s1"></span>measured angle (q)'
                '<span class="key s2"></span>command (u), lined up with the state it acts on · rad · grey lines = phase starts</p>')
    for name, joints in GROUPS:
        charts = []
        for j in joints:
            cmd = np.full(n, np.nan)
            cmd[: n - s] = r[u(j)].to_numpy(np.float64)[s:]
            charts.append(chart(t, [("q", r[q(j)].to_numpy(np.float64), 1), ("u", cmd, 2)], bounds, short(j), "rad", 110, legend=True))
        body.append(f'<h4>{name}</h4><div class="grid">{"".join(charts)}</div>')

    # Walker command, waist lock, IMU
    charts = []
    if side is not None:
        sr = side.rows
        ts = np.arange(len(sr)) / fps
        charts.append(chart(ts, [("vx", sr["cmd_vx"], 1), ("vy", sr["cmd_vy"], 2), ("yaw rate", sr["cmd_yaw"], 3)], bounds,
                            "Walker command (m/s, rad/s)", "", 150, steps=True))
        charts.append(chart(ts, [("waist lock", sr["waist_lock"], 1)], bounds, "Waist lock (1 = held by the script)", "", 150, steps=True))
    for k in IMU:
        v = np.stack(r[k].to_numpy())
        charts.append(chart(t, [(a, v[:, i], i + 1) for i, a in enumerate("xyz")], bounds, k, "", 150))
    body.append(f'<h3>Walker, waist, IMU</h3><div class="grid wide">{"".join(charts)}</div>')

    # Box and pelvis (privileged poses: x, y, z, then the quaternion; MuJoCo frame, z up)
    poses = [c for c in r.columns if c.startswith("pose.")]
    if poses:
        charts = []
        pel = np.stack(r["pose.pelvis"].to_numpy()) if "pose.pelvis" in r else None
        for c in poses:
            if c == "pose.pelvis":
                continue
            p = np.stack(r[c].to_numpy())
            charts.append(chart(t, [("box z", p[:, 2], 1)] + ([("pelvis z", pel[:, 2], 2)] if pel is not None else []), bounds,
                                f"{c.removeprefix('pose.box.')}: height (m)", "m", 150))
            if pel is not None:
                charts.append(chart(t, [("box-pelvis distance", np.linalg.norm(p[:, :2] - pel[:, :2], axis=1), 1)], bounds,
                                    f"{c.removeprefix('pose.box.')}: horizontal distance from the pelvis (m)", "m", 150))
        if pel is not None:
            charts.append(chart(t, [("pelvis x", pel[:, 0], 1), ("pelvis y", pel[:, 1], 2)], bounds, "Pelvis position x, y (m)", "m", 150))
        body.append(f'<h3>Box and pelvis</h3><p class="legend">Privileged poses, for checking only (never a policy input). MuJoCo frame, z up.</p>'
                    f'<div class="grid wide">{"".join(charts)}</div>')
    return f'<details class="episode"{" open" if is_open else ""}>{head}{"".join(body)}</details>'


ICON = {"pass": "✓", "warn": "!", "fail": "✕"}
LABEL = {"pass": "Pass", "warn": "Warning", "fail": "Fail"}

CSS = """
:root{--surface:#fcfcfb;--panel:#f3f2ef;--ink:#0b0b0b;--ink2:#52514e;--muted:#6f6e69;--rule:#dcdad4;--s1:#2a78d6;--s2:#eb6834;--s3:#1baf7a;
--good:#0ca30c;--warning:#b37800;--critical:#d03b3b;--p0:#e9e7e2;--p1:#d9d6cf}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--surface:#1a1a19;--panel:#242422;--ink:#f0efec;--ink2:#c3c2b7;--muted:#9a9990;
--rule:#3a3a37;--s1:#3987e5;--s2:#d95926;--s3:#199e70;--good:#0ca30c;--warning:#fab219;--critical:#e66767;--p0:#2c2c2a;--p1:#383835}}
:root[data-theme="dark"]{--surface:#1a1a19;--panel:#242422;--ink:#f0efec;--ink2:#c3c2b7;--muted:#9a9990;--rule:#3a3a37;--s1:#3987e5;--s2:#d95926;
--s3:#199e70;--good:#0ca30c;--warning:#fab219;--critical:#e66767;--p0:#2c2c2a;--p1:#383835}
*{box-sizing:border-box}body{margin:0;background:var(--surface);color:var(--ink);font:14px/1.45 system-ui,-apple-system,"Segoe UI",sans-serif}
main{max-width:1180px;margin:0 auto;padding:24px 16px 64px}h1{font-size:22px;margin:0 0 4px}h2{font-size:17px;margin:28px 0 8px}
h3{font-size:15px;margin:20px 0 6px}h4{font-size:13px;margin:12px 0 4px;color:var(--ink2)}.muted,.legend{color:var(--muted);font-size:12.5px}
code{font-size:12px;background:var(--panel);padding:0 3px;border-radius:3px;overflow-wrap:anywhere}
table{border-collapse:collapse;width:100%;font-size:12.5px;margin:8px 0}th,td{text-align:left;padding:4px 6px;border-bottom:1px solid var(--rule);vertical-align:top}
th{color:var(--ink2);font-weight:600}.badge{white-space:nowrap;font-weight:600;font-size:12px}.badge.pass{color:var(--good)}
.badge.warn{color:var(--warning)}.badge.fail{color:var(--critical)}
details.episode{border:1px solid var(--rule);border-radius:8px;padding:8px 12px;margin:10px 0;background:var(--surface)}
summary{cursor:pointer;font-weight:600}.timeline{display:flex;height:26px;margin:10px 0;border-radius:4px;overflow:hidden;gap:2px}
.seg{display:flex;align-items:center;justify-content:center;min-width:2px;font-size:11px;color:var(--ink2);overflow:hidden;white-space:nowrap}
.seg.p0{background:var(--p0)}.seg.p1{background:var(--p1)}.thumbs{display:flex;flex-wrap:wrap;gap:6px}
.thumb{margin:0}.thumb img{display:block;width:160px;height:auto;border-radius:4px}.thumb figcaption{font-size:11px;color:var(--muted)}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(230px,1fr));gap:6px 12px}.grid.wide{grid-template-columns:repeat(auto-fill,minmax(340px,1fr))}
.chart{margin:0;position:relative}.chart figcaption{font-size:12px;color:var(--ink2)}.chart svg{width:100%;height:auto;display:block;overflow:visible}
.grid line.grid{stroke:var(--rule);stroke-width:1}svg .grid{stroke:var(--rule)}svg .phase{stroke:var(--rule);stroke-dasharray:2 3}
svg .tick{fill:var(--muted);font-size:9px}svg polyline{fill:none;stroke-width:1.5;stroke-linejoin:round}
svg .s1{stroke:var(--s1)}svg .s2{stroke:var(--s2)}svg .s3{stroke:var(--s3)}svg .cross{stroke:var(--ink2);stroke-width:1;visibility:hidden}
.key{display:inline-block;width:14px;height:2px;margin:0 5px 3px 10px;vertical-align:middle}.key.s1{background:var(--s1)}.key.s2{background:var(--s2)}
.key.s3{background:var(--s3)}#tip{position:fixed;pointer-events:none;background:var(--panel);color:var(--ink);border:1px solid var(--rule);
border-radius:6px;padding:4px 8px;font-size:12px;display:none;z-index:9;white-space:nowrap}#tip i{display:inline-block;width:8px;height:8px;
border-radius:50%;margin-right:5px}.summary td:first-child{white-space:nowrap}
"""

JS = """
const tip=document.getElementById('tip');
document.querySelectorAll('svg[data-c]').forEach(svg=>{
  const c=JSON.parse(svg.dataset.c), cross=svg.querySelector('.cross');
  svg.addEventListener('mousemove',e=>{
    const r=svg.getBoundingClientRect(), vb=svg.viewBox.baseVal, x=(e.clientX-r.left)*vb.width/r.width;
    const [l,rt]=c.box, f=Math.min(1,Math.max(0,(x-l)/(rt-l))), tt=c.t[0]+f*(c.t[1]-c.t[0]);
    let i=0,b=1e9; c.x.forEach((v,k)=>{const d=Math.abs(v-tt); if(d<b){b=d;i=k}});
    const px=l+(c.x[i]-c.t[0])/(c.t[1]-c.t[0])*(rt-l); cross.setAttribute('x1',px); cross.setAttribute('x2',px); cross.style.visibility='visible';
    tip.innerHTML=`<b>${c.x[i].toFixed(2)} s</b><br>`+c.s.map(([n,s,v])=>`<i style="background:var(--s${s})"></i>${n} ${v[i]==null||isNaN(v[i])?'–':(+v[i]).toFixed(3)} ${c.u}`).join('<br>');
    tip.style.display='block'; tip.style.left=Math.min(e.clientX+14,innerWidth-tip.offsetWidth-8)+'px'; tip.style.top=(e.clientY+14)+'px';
  });
  svg.addEventListener('mouseleave',()=>{tip.style.display='none'; cross.style.visibility='hidden'});
});
"""


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", required=True)
    ap.add_argument("--sidecar", help="the run's sidecar folder (without it: no phases, walker command or alignment)")
    ap.add_argument("--mjcf", required=True, help="robot MJCF, for the joint ranges")
    ap.add_argument("--cameras", nargs="*", help="camera Output Names (default: every stream in the session)")
    ap.add_argument("--episodes", nargs="*", type=int, help="only these episodes")
    ap.add_argument("--out", help="HTML file (default: inspect_<session folder>.html here)")
    a = ap.parse_args()

    info, episodes = read_session(a.session)
    fps = float(info.get("fps", 50))
    ranges = mjcf_ranges(a.mjcf)
    lo = np.array([ranges.get(j, (-np.inf, np.inf))[0] for j in UPPER_JOINTS]) - 1e-6
    hi = np.array([ranges.get(j, (-np.inf, np.inf))[1] for j in UPPER_JOINTS]) + 1e-6
    streams = sorted({s for ep in episodes for s in ep.videos})
    cams = [f"observation.images.{c}" for c in a.cameras] if a.cameras is not None else streams
    episodes = [ep for ep in episodes if a.episodes is None or ep.index in a.episodes]

    sections, summary, shifts = [], [], {}
    for k, ep in enumerate(episodes):
        side = read_sidecar(a.sidecar, ep.index) if a.sidecar else None
        frames = {c.removeprefix("observation.images."): (np.stack(list(video_frames(ep.videos[c], ep.video_start.get(c, 0.0), fps, *THUMB)))
                                                         if c in ep.videos else None) for c in cams}
        checks, extra = check_episode(ep, side, fps, lo, hi, frames)
        if side is not None:
            shifts[ep.index] = extra["shift"]
        sections.append(episode_html(ep, side, checks, extra, fps, frames, k == 0))
        verdict = "fail" if any(c[0] == "fail" for c in checks) else "warn" if any(c[0] == "warn" for c in checks) else "pass"
        summary.append((ep.index, len(ep.rows) / fps, side.header.get("success") if side else None,
                        (side.header.get("fail_reason") if side else "") or "", verdict, extra["shift"] if side else None))
        print(f"episode {ep.index}: {verdict}")

    # Session verdict: every episode's worst check, and one action shift for all
    one_shift = len(set(shifts.values())) <= 1
    worst = "fail" if any(s[4] == "fail" for s in summary) or not one_shift else "warn" if any(s[4] == "warn" for s in summary) else "pass"
    ok = sum(1 for s in summary if s[2])
    rows = "".join(f'<tr><td>{i}</td><td><span class="badge {v}">{ICON[v]} {LABEL[v]}</span></td><td>{sec:.1f} s</td>'
                   f'<td>{"yes" if suc else "no" if suc is not None else "–"}</td><td>{"–" if sh is None else sh}</td><td>{esc(fr)}</td></tr>'
                   for i, sec, suc, fr, v, sh in summary)
    title = f"Inspect {Path(a.session).name}"
    page = f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>{esc(title)}</title><style>{CSS}</style></head><body><main>
<h1>{esc(title)}</h1>
<p class="muted">Session <code>{esc(Path(a.session).resolve())}</code><br>Sidecar <code>{esc(Path(a.sidecar).resolve()) if a.sidecar else "none"}</code><br>
{len(summary)} episodes at {fps:.0f} Hz · cameras {", ".join(c.removeprefix("observation.images.") for c in cams) or "none"}</p>
<h2><span class="badge {worst}">{ICON[worst]} {LABEL[worst]}</span> {ok} of {len(summary)} episodes succeeded · action shift
{", ".join(sorted({str(v) for v in shifts.values()})) or "–"} row(s){"" if one_shift else " (episodes disagree: a fail)"}</h2>
<p class="muted">Fail = the data is broken and would be wrong for training. Warning = worth a look; the data may still be usable.
Hover a chart to read values.</p>
<table class="summary"><thead><tr><th>Episode</th><th>Data</th><th>Length</th><th>Expert succeeded</th><th>Action shift</th><th>Fail reason</th></tr></thead>
<tbody>{rows}</tbody></table>
<h2>Episodes</h2>{"".join(sections)}
</main><div id="tip"></div><script>{JS}</script></body></html>"""
    out = Path(a.out or f"inspect_{Path(a.session).name}.html")
    out.write_text(page)
    print(f"wrote {out} ({out.stat().st_size / 1e6:.1f} MB): {LABEL[worst].lower()}, {ok}/{len(summary)} succeeded")


if __name__ == "__main__":
    main()
