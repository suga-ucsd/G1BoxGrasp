"""Writes g1_grasp_map.html: an interactive map of every function in the G1BoxGrasp controller (src/), read from the code itself.
Each function: file and line, signature, its comment, what it calls and what calls it, the engine API it uses, the settings and
state it reads and writes, its Fail messages, warnings and threshold checks; plus the phase flow from OnUpdate.

    python3 tools/make_map.py            # from the repo root; rerun after changing src/
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "src"
OUT = ROOT / "g1_grasp_map.html"
FOLDERS = ["", "Config", "Core", "Phases", "Hands", "Body", "World", "Diagnostics", "Recording"]
FOLDER_NAMES = {"": "Entry"}
DECL = re.compile(r"^ {8}(private|public|protected|internal|static|const|readonly|sealed|enum|class)\b")
LEAD = re.compile(r"^ {8}(//|\[)")


def _code(text: str, i: int, out: list[str], hole: bool = False) -> int:
    """Copies code from i to out (comments dropped, strings via _string); in an interpolation hole, stops at its closing brace
    and skips a format specifier (a ':' outside brackets). Returns the index after what it read."""
    n, depth = len(text), 0
    while i < n:
        c = text[i]
        if text.startswith("//", i):
            j = text.find("\n", i)
            i = n if j < 0 else j
        elif text.startswith("/*", i):
            i = text.find("*/", i) + 2
        elif c == "'":
            j = i + 1 + (text[i + 1] == "\\")
            i = text.find("'", j + 1) + 1
            out.append(" ")
        elif c == '"' or (c in "$@" and text[i + 1 : i + 2] in ('"', "$", "@") and '"' in text[i + 1 : i + 3]):
            i = _string(text, i, out)
        elif hole and c in "([":
            depth += 1
            out.append(c)
            i += 1
        elif hole and c in ")]":
            depth -= 1
            out.append(c)
            i += 1
        elif hole and c == "{":
            depth += 1
            out.append(c)
            i += 1
        elif hole and c == "}":
            if depth == 0:
                return i + 1
            depth -= 1
            out.append(c)
            i += 1
        elif hole and c == ":" and depth == 0:  # format specifier up to the hole's end
            return text.index("}", i) + 1
        else:
            out.append(c)
            i += 1
    return i


def _string(text: str, i: int, out: list[str] | None = None) -> int:
    """Reads the string literal at i (plain, verbatim, interpolated); its holes' code goes to out. Returns the index after it."""
    q = text.index('"', i)
    interp, verbatim = "$" in text[i:q], "@" in text[i:q]
    i, n = q + 1, len(text)
    while i < n:
        c = text[i]
        if c == "\\" and not verbatim:
            i += 2
        elif c == '"':
            if verbatim and text[i + 1 : i + 2] == '"':
                i += 2
                continue
            if out is not None:
                out.append('""')
            return i + 1
        elif interp and c == "{":
            if text[i + 1 : i + 2] == "{":
                i += 2
                continue
            hole: list[str] = []
            i = _code(text, i + 1, hole, hole=True)
            if out is not None:
                out.append(" (" + "".join(hole) + ") ")
        else:
            i += 1
    return i


def code_only(text: str) -> str:
    """Comments removed and string-literal text blanked (interpolation holes kept), so names in strings aren't calls."""
    out: list[str] = []
    _code(text, 0, out)
    return "".join(out)


def strings(text: str, call: str) -> list[str]:
    """First string argument of every call(...) (interpolated or not), whitespace collapsed."""
    found = []
    for m in re.finditer(rf"(?<![\w.]){re.escape(call)}\(\s*(?=[$@]*\")", text):
        start = m.end()
        end = _string(text, start)
        lit = text[start:end]
        found.append(re.sub(r"\s+", " ", lit[lit.index('"') + 1 : -1]).strip())
    return found


def members(path: Path) -> tuple[str, list[dict]]:
    lines = path.read_text().split("\n")
    cls = next(i for i, l in enumerate(lines) if "partial class G1BoxGrasp" in l)
    about = " ".join(l.strip()[3:] for l in lines[:cls] if l.startswith("    //"))
    close = max(i for i, l in enumerate(lines) if l == "    }")
    starts = []
    for i in range(cls + 2, close):
        if DECL.match(lines[i]):
            j = i
            while LEAD.match(lines[j - 1]):
                j -= 1
            starts.append((j, i))
    out = []
    for k, (j, i) in enumerate(starts):
        last = (starts[k + 1][0] if k + 1 < len(starts) else close) - 1
        lead = [l.strip() for l in lines[j:i]]
        comment = " ".join(re.sub(r"^///?\s?|</?summary>", "", l).strip() for l in lead if l.startswith("//"))
        decl = []
        for t in range(i, last + 1):
            s = re.sub(r"\s*//.*$", "", lines[t]).strip()
            if s == "{" or s.startswith("=>"):
                break
            decl.append(s)
            if "=>" in s or s.endswith(";") or s.endswith("{"):
                break
        sig = " ".join(decl)
        head = sig.split("=>")[0]
        if m := re.search(r"\b(enum|class|struct)\s+(\w+)", sig):
            kind, name = "type", m.group(2)
        elif (mm := re.search(r"(\w+)\s*(<[^>]*>)?\s*\(", head)) and "=" not in head.split("(")[0] and not re.search(r"<\s*\(|\breadonly\s*\(", head):
            kind, name = "method", re.search(r"(\w+)\s*(<[^>]*>)?\s*\(", head).group(1)
            sig = re.sub(r"\s*\{$", "", head).strip()
        elif "=>" in sig and "(" not in head.split("=>")[0]:
            kind, name = "property", re.search(r"(\w+)\s*$", head.strip()).group(1)
            sig = head.strip()
        else:
            kind, name = "field", (re.search(r"(\w+)\s*(=|;|,)", sig) or re.search(r"(\w+)", sig)).group(1)
        body = "\n".join(lines[i : last + 1])
        out.append({"name": name, "kind": kind, "sig": sig, "comment": comment, "line": i + 1, "body": body,
                    "public": sig.startswith("public ")})
    return about, out


def main() -> int:
    files, nodes, fields, settings = [], {}, set(), set()
    for path in sorted(SRC.rglob("*.cs")):
        rel = path.relative_to(SRC).as_posix()
        folder = rel.split("/")[0] if "/" in rel else ""
        about, mem = members(path)
        files.append({"file": rel, "folder": folder, "about": about, "lines": len(path.read_text().split("\n")) - 1, "members": []})
        for m in mem:
            if m["kind"] == "field":
                (settings if m["public"] else fields).add(m["name"])
                if m["kind"] == "field" and not m["public"]:
                    for extra in re.findall(r"\b(m_\w+|s_\w+|k_\w+)\b", m["sig"]):
                        fields.add(extra)
                continue
            m.update(file=rel, folder=folder)
            if m["name"] in nodes:  # overload or same name: keep both under one node
                nodes[m["name"]]["body"] += "\n" + m["body"]
                continue
            nodes[m["name"]] = m
            files[-1]["members"].append(m["name"])
    names = set(nodes)

    entity_fields = {s for s in settings if re.search(rf"public Entity\??\s+{s}\b", "\n".join(p.read_text() for p in SRC.rglob("*.cs")))}
    for n in nodes.values():
        raw = n["body"]
        code = code_only(raw.split("\n", 1)[1] if n["kind"] == "method" and "\n" in raw else raw)
        n["calls"] = sorted({c for c in re.findall(r"(?<![\w.])([A-Z]\w*)\b", code) if c in names and c != n["name"] and nodes[c]["kind"] != "type"})
        api = set()
        for pat, tag in [(r"m_Mujoco!?\s*\.(\w+)", "Mujoco"), (r"m_Robot!?\s*\.(\w+)", "Robot"), (r"(?<![\w.])Scene\.(\w+)", "Scene"),
                         (r"(?<![\w.])Observer\.(\w+)", "Observer"), (r"GetComponent<(\w+)>", "GetComponent")]:
            api |= {f"{tag}.{x}" if tag != "GetComponent" else f"GetComponent<{x}>" for x in re.findall(pat, code)}
        for e in entity_fields:
            api |= {f"{e}.{x}" for x in re.findall(rf"(?<![\w.]){e}!?\s*\.(\w+)", code)}
        n["api"] = sorted(api)
        n["settings"] = sorted({s for s in re.findall(r"(?<![\w.])([A-Z]\w*)\b", code) if s in settings})
        writes = set(re.findall(r"\b(m_\w+)(?:\[[^\]]*\])?\s*(?:=(?!=)|\+=|-=|\*=|/=|\+\+|--)", code))
        writes |= set(re.findall(r"(?:\+\+|--|\bref |\bout )\s*(m_\w+)", code))
        n["writes"] = sorted(w for w in writes if w in fields)
        n["reads"] = sorted({r for r in re.findall(r"\b(m_\w+)\b", code) if r in fields} - writes)
        n["fails"] = strings(raw, "Fail")
        n["warns"] = [w[:160] for w in strings(raw, "Log.Warn")]
        checks, rl = [], raw.split("\n")
        for t, l in enumerate(rl):
            s = l.strip()
            if not re.match(r"^(else\s+)?(if|while)\s*\(", s):
                continue
            cond = s
            if s.endswith("("):
                parts = []
                for u in rl[t + 1 :]:
                    if u.strip() == ")":
                        break
                    parts.append(u.strip())
                cond = s + " ".join(parts) + ")"
            cond = re.sub(r"\s+", " ", cond)
            if re.search(r"[<>]=?", code_only(cond)) and re.search(r"\bk_\w+|" + "|".join(rf"\b{x}\b" for x in settings), cond):
                checks.append(cond[:200])
        n["checks"] = checks
        n["advance"] = sorted({p for a in re.findall(r"Advance\(([^;]*?)\)\s*;", code) for p in re.findall(r"Phase\.(\w+)", a)})
    for n in nodes.values():
        n["callers"] = sorted(m["name"] for m in nodes.values() if n["name"] in m["calls"])

    # Phase flow: each case of OnUpdate's switch, what it runs and where it can go next
    upd = nodes["OnUpdate"]["body"]
    phases_enum = re.findall(r"^\s{12}(\w+),?\s*(?://.*)?$", nodes["Phase"]["body"], re.M)
    flow = []
    cases = re.split(r"\n\s*case Phase\.(\w+):", upd)
    for k in range(1, len(cases), 2):
        ph, block = cases[k], code_only(cases[k + 1].split("default:")[0])
        runs = [c for c in dict.fromkeys(re.findall(r"(?<![\w.])([A-Z]\w*)\s*\(", block)) if c in names and c not in ("Advance", "NextStep")]
        nxt = {p for a in re.findall(r"Advance\(([^;]*?)\)\s*;", block) for p in re.findall(r"Phase\.(\w+)", a)}
        seen, todo, fails = set(), [(r, 0) for r in runs], "Fail(" in block
        while todo:  # Advance targets in the functions the case runs, three calls deep (Fail's own targets listed apart)
            r, d = todo.pop()
            if r in seen or d > 3:
                continue
            seen.add(r)
            if r == "Fail":
                fails = True
                continue
            nxt |= set(nodes[r]["advance"])
            todo += [(c, d + 1) for c in nodes[r]["calls"] if c != "OnUpdate"]
        flow.append({"phase": ph, "runs": runs, "next": sorted(nxt - {ph}), "fails": fails})

    for n in nodes.values():
        n.pop("body")
    data = {"folders": [{"id": f, "name": FOLDER_NAMES.get(f, f)} for f in FOLDERS], "files": files, "nodes": nodes, "flow": flow,
            "phases": phases_enum}
    nf = sum(1 for n in nodes.values() if n["kind"] != "type")
    edges = sum(len(n["calls"]) for n in nodes.values())
    html = TEMPLATE.replace("/*DATA*/null", json.dumps(data, separators=(",", ":"))).replace("{{STATS}}", f"{len(files)} files · {nf} functions and properties · {edges} call edges")
    OUT.write_text(html)
    print(f"wrote {OUT.relative_to(ROOT)}: {len(files)} files, {nf} functions, {edges} call edges, {len(flow)} phases")
    return 0


TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>G1BoxGrasp Function Map</title>
<style>
:root {
  --bg: #f6f7f9; --panel: #ffffff; --ink: #1c2330; --muted: #5d6778; --line: #d5dae3; --chip: #eef1f5; --code: #f1f3f6;
  --out: #d9480f; --in: #1971c2; --sel: #111827;
  --g0: #7048e8; --g1: #868e96; --g2: #0c8599; --g3: #1971c2; --g4: #c2255c; --g5: #2f9e44; --g6: #e67700; --g7: #5f3dc4; --g8: #a61e4d;
}
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) {
  --bg: #0f1218; --panel: #171b23; --ink: #e6e9ef; --muted: #98a2b3; --line: #2a303b; --chip: #212733; --code: #1d222c;
  --out: #ff8a4c; --in: #5aa9ff; --sel: #ffffff;
  --g0: #9775fa; --g1: #adb5bd; --g2: #3bc9db; --g3: #4dabf7; --g4: #f783ac; --g5: #69db7c; --g6: #ffa94d; --g7: #b197fc; --g8: #ff8787;
} }
:root[data-theme="dark"] {
  --bg: #0f1218; --panel: #171b23; --ink: #e6e9ef; --muted: #98a2b3; --line: #2a303b; --chip: #212733; --code: #1d222c;
  --out: #ff8a4c; --in: #5aa9ff; --sel: #ffffff;
  --g0: #9775fa; --g1: #adb5bd; --g2: #3bc9db; --g3: #4dabf7; --g4: #f783ac; --g5: #69db7c; --g6: #ffa94d; --g7: #b197fc; --g8: #ff8787;
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--bg); color: var(--ink); font: 14px/1.5 system-ui, -apple-system, "Segoe UI", sans-serif; }
header, main { max-width: 1500px; margin: 0 auto; padding: 0 16px; }
header { padding-top: 20px; }
h1 { font-size: 20px; margin: 0 0 4px; }
h2 { font-size: 16px; margin: 28px 0 8px; }
.sub { color: var(--muted); margin: 0 0 12px; max-width: 980px; }
code, .mono { font: 12.5px/1.45 ui-monospace, "DejaVu Sans Mono", monospace; }
.controls { display: flex; flex-wrap: wrap; gap: 8px 16px; align-items: center; margin: 8px 0 12px; }
.controls input[type=search] { flex: 1 1 260px; max-width: 420px; padding: 7px 10px; border: 1px solid var(--line); border-radius: 6px; background: var(--panel); color: var(--ink); font: inherit; }
.controls label { color: var(--muted); font-size: 13px; }
.layout { display: grid; grid-template-columns: minmax(0, 1fr) 380px; gap: 16px; align-items: start; }
@media (max-width: 1000px) { .layout { grid-template-columns: 1fr; } }
.map { position: relative; display: grid; grid-template-columns: repeat(auto-fill, minmax(190px, 1fr)); gap: 12px; }
.col h3 { font-size: 12px; text-transform: uppercase; letter-spacing: .06em; margin: 0 0 6px; color: var(--muted); display: flex; align-items: center; gap: 6px; }
.dot { width: 9px; height: 9px; border-radius: 50%; display: inline-block; }
.card { background: var(--panel); border: 1px solid var(--line); border-left: 3px solid var(--gc); border-radius: 6px; padding: 6px 7px 7px; margin-bottom: 10px; }
.card .fn { font-size: 12px; font-weight: 600; margin-bottom: 4px; word-break: break-all; }
.card .ab { font-size: 11.5px; color: var(--muted); margin: -2px 0 5px; line-height: 1.35; }
.chips { display: flex; flex-wrap: wrap; gap: 3px; }
.chip { font: 11.5px/1.3 ui-monospace, "DejaVu Sans Mono", monospace; padding: 2px 5px; border-radius: 4px; background: var(--chip); border: 1px solid transparent; color: var(--ink); cursor: pointer; }
.chip.prop { font-style: italic; }
.chip:hover, .chip:focus-visible { border-color: var(--gc); outline: none; }
.chip.sel { background: var(--sel); color: var(--panel); }
.chip.callee { border-color: var(--out); box-shadow: inset 0 0 0 1px var(--out); }
.chip.caller { border-color: var(--in); box-shadow: inset 0 0 0 1px var(--in); }
.chip.dim { opacity: .28; }
.chip.hit { outline: 2px solid var(--out); }
svg.edges { position: absolute; inset: 0; width: 100%; height: 100%; pointer-events: none; overflow: visible; }
.panel { position: sticky; top: 12px; background: var(--panel); border: 1px solid var(--line); border-radius: 8px; padding: 12px 14px; max-height: calc(100vh - 24px); overflow: auto; }
@media (max-width: 1000px) { .panel { position: static; max-height: none; } }
.panel h3 { margin: 0 0 2px; font-size: 16px; word-break: break-all; }
.panel .where { color: var(--muted); font-size: 12px; margin-bottom: 8px; }
.panel pre { background: var(--code); padding: 6px 8px; border-radius: 5px; white-space: pre-wrap; word-break: break-word; margin: 0 0 8px; font: 12px/1.4 ui-monospace, "DejaVu Sans Mono", monospace; }
.panel h4 { font-size: 12px; text-transform: uppercase; letter-spacing: .05em; color: var(--muted); margin: 12px 0 4px; }
.panel ul { margin: 0; padding-left: 18px; }
.panel li { margin: 2px 0; font-size: 12.5px; }
.hint { color: var(--muted); }
.key { display: inline-flex; gap: 12px; font-size: 12px; color: var(--muted); }
.key span::before { content: ""; display: inline-block; width: 14px; height: 2px; vertical-align: middle; margin-right: 4px; background: currentColor; }
.key .o { color: var(--out); } .key .i { color: var(--in); }
.tablewrap { overflow-x: auto; background: var(--panel); border: 1px solid var(--line); border-radius: 8px; }
table { border-collapse: collapse; width: 100%; font-size: 12.5px; }
th, td { text-align: left; vertical-align: top; padding: 5px 8px; border-bottom: 1px solid var(--line); }
th { position: sticky; top: 0; background: var(--panel); }
td .chip { display: inline-block; margin: 1px 2px; }
a { color: var(--in); }
button.theme { margin-left: auto; padding: 5px 10px; border: 1px solid var(--line); border-radius: 6px; background: var(--panel); color: var(--ink); font: inherit; font-size: 12px; cursor: pointer; }
footer { color: var(--muted); font-size: 12px; padding: 24px 16px 32px; max-width: 1500px; margin: 0 auto; }
</style>
</head>
<body>
<header>
  <h1>G1BoxGrasp: function map</h1>
  <p class="sub">Every function and computed property of the <code>G1Pipe.G1BoxGrasp</code> controller in <code>src/</code>, grouped by folder and file ({{STATS}}). Click one to see its signature and comment, what it calls (orange) and what calls it (blue), the engine API it uses, the settings and state it reads and writes, and its failure messages and threshold checks. Generated from the code by <code>tools/make_map.py</code>.</p>
</header>
<main>
  <div class="controls">
    <input type="search" id="q" placeholder="Find a function, setting, field or engine call…" aria-label="Find">
    <label><input type="checkbox" id="hideDiag"> Hide Diagnostics</label>
    <span class="key"><span class="o">calls</span><span class="i">called by</span></span>
    <button class="theme" id="theme" type="button">Theme</button>
  </div>
  <div class="layout">
    <div class="map" id="map"></div>
    <aside class="panel" id="panel"><p class="hint">Select a function. Start with <code>OnUpdate</code>: it runs one phase step per frame.</p></aside>
  </div>
  <h2>Phase flow (from OnUpdate)</h2>
  <p class="sub">Each phase's case in <code>OnUpdate</code>, the functions it runs, and the phases those can move to (three calls deep). A phase that can fail goes through <code>Fail</code> to <code>Recover</code>, or to <code>Holding</code> while a box is being carried.</p>
  <div class="tablewrap"><table id="flow"></table></div>
  <h2>Engine API used</h2>
  <div class="tablewrap"><table id="api"></table></div>
  <h2>Failure messages</h2>
  <div class="tablewrap"><table id="fails"></table></div>
  <h2>Files</h2>
  <div class="tablewrap"><table id="files"></table></div>
</main>
<footer>Regenerate after changing the code: <code>python3 tools/make_map.py</code> from the repo root.</footer>
<script>
const D = /*DATA*/null;
const N = D.nodes, G = Object.fromEntries(D.folders.map((f, i) => [f.id, i]));
const $ = s => document.querySelector(s);
const esc = s => String(s).replace(/[&<>"]/g, c => ({"&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;"}[c]));
const color = f => `var(--g${G[f] ?? 1})`;
let sel = null;
const chipEls = {};

function buildMap() {
  const map = $("#map");
  map.innerHTML = '<svg class="edges" id="edges" aria-hidden="true"></svg>';
  for (const fo of D.folders) {
    const fl = D.files.filter(f => f.folder === fo.id && f.members.length);
    if (!fl.length) continue;
    const col = document.createElement("div");
    col.className = "col"; col.dataset.folder = fo.id;
    col.innerHTML = `<h3><span class="dot" style="background:${color(fo.id)}"></span>${esc(fo.name)}</h3>`;
    for (const f of fl) {
      const card = document.createElement("div");
      card.className = "card"; card.style.setProperty("--gc", color(fo.id));
      card.innerHTML = `<div class="fn">${esc(f.file)}</div>${f.about ? `<div class="ab" title="${esc(f.about)}">${esc(f.about.split(/(?<=\.) /)[0])}</div>` : ""}<div class="chips"></div>`;
      const chips = card.querySelector(".chips");
      for (const m of f.members) {
        const n = N[m], b = document.createElement("button");
        b.type = "button"; b.className = "chip" + (n.kind === "property" ? " prop" : "");
        b.textContent = m; b.title = n.comment || n.sig; b.style.setProperty("--gc", color(fo.id));
        b.addEventListener("click", () => select(m));
        chips.appendChild(b); chipEls[m] = b;
      }
      col.appendChild(card);
    }
    map.appendChild(col);
  }
}

function list(title, items, fn = x => esc(x)) {
  return items && items.length ? `<h4>${title} (${items.length})</h4><ul>${items.map(x => `<li>${fn(x)}</li>`).join("")}</ul>` : "";
}
const link = m => `<a href="#" data-go="${esc(m)}"><code>${esc(m)}</code></a>`;

function select(name) {
  sel = name;
  const n = N[name];
  for (const [m, el] of Object.entries(chipEls)) {
    el.classList.toggle("sel", m === name);
    el.classList.toggle("callee", n.calls.includes(m));
    el.classList.toggle("caller", n.callers.includes(m));
    el.classList.toggle("dim", m !== name && !n.calls.includes(m) && !n.callers.includes(m));
  }
  $("#panel").innerHTML = `<h3>${esc(name)}</h3><div class="where">src/${esc(n.file)}:${n.line} · ${n.kind}</div>
    <pre>${esc(n.sig)}</pre>${n.comment ? `<p>${esc(n.comment)}</p>` : ""}
    ${list("Calls", n.calls, link)}${list("Called by", n.callers, link)}
    ${list("Moves to phase", n.advance)}${list("Engine API", n.api, x => `<code>${esc(x)}</code>`)}
    ${list("Settings read", n.settings, x => `<code>${esc(x)}</code>`)}${list("State written", n.writes, x => `<code>${esc(x)}</code>`)}
    ${list("State read", n.reads, x => `<code>${esc(x)}</code>`)}${list("Fails with", n.fails)}${list("Warnings", n.warns)}
    ${list("Threshold checks", n.checks, x => `<code>${esc(x)}</code>`)}
    <p class="hint" style="margin-top:12px"><a href="#" id="clear">Clear selection</a></p>`;
  drawEdges();
  if (window.matchMedia("(max-width: 1000px)").matches) $("#panel").scrollIntoView({behavior: "smooth", block: "start"});
}

function clearSel() {
  sel = null;
  for (const el of Object.values(chipEls)) el.classList.remove("sel", "callee", "caller", "dim");
  $("#panel").innerHTML = '<p class="hint">Select a function. Start with <code>OnUpdate</code>: it runs one phase step per frame.</p>';
  drawEdges();
}

function drawEdges() {
  const svg = $("#edges");
  if (!svg) return;
  svg.innerHTML = "";
  if (!sel || !chipEls[sel] || !chipEls[sel].offsetParent) return;
  const box = $("#map").getBoundingClientRect();
  const c = el => { const r = el.getBoundingClientRect(); return [r.left + r.width / 2 - box.left, r.top + r.height / 2 - box.top]; };
  const [sx, sy] = c(chipEls[sel]);
  const n = N[sel];
  const path = (a, b, col) => {
    const [x1, y1] = a, [x2, y2] = b, dx = (x2 - x1) / 2;
    const p = document.createElementNS("http://www.w3.org/2000/svg", "path");
    p.setAttribute("d", `M${x1},${y1} C${x1 + dx},${y1} ${x2 - dx},${y2} ${x2},${y2}`);
    p.setAttribute("fill", "none"); p.setAttribute("stroke", col); p.setAttribute("stroke-width", "1.4"); p.setAttribute("opacity", ".75");
    svg.appendChild(p);
  };
  for (const m of n.calls) if (chipEls[m]?.offsetParent) path([sx, sy], c(chipEls[m]), "var(--out)");
  for (const m of n.callers) if (chipEls[m]?.offsetParent) path(c(chipEls[m]), [sx, sy], "var(--in)");
}

function search() {
  const q = $("#q").value.trim().toLowerCase();
  for (const [m, el] of Object.entries(chipEls)) {
    const n = N[m];
    const hay = [m, n.comment, n.sig, ...n.api, ...n.settings, ...n.writes, ...n.reads].join(" ").toLowerCase();
    el.classList.toggle("hit", !!q && hay.includes(q));
  }
}

function tables() {
  const chipLink = m => `<a href="#" class="chip" data-go="${esc(m)}">${esc(m)}</a>`;
  $("#flow").innerHTML = "<thead><tr><th>Phase</th><th>Runs each frame</th><th>Can move to</th><th>Can fail</th></tr></thead><tbody>" +
    D.flow.map(f => `<tr><td><code>${esc(f.phase)}</code></td><td>${f.runs.map(chipLink).join(" ")}</td><td>${f.next.map(x => `<code>${esc(x)}</code>`).join(", ")}</td><td>${f.fails ? "yes" : ""}</td></tr>`).join("") + "</tbody>";
  const api = {};
  for (const [m, n] of Object.entries(N)) for (const a of n.api) (api[a] ??= []).push(m);
  $("#api").innerHTML = "<thead><tr><th>Call</th><th>Used by</th></tr></thead><tbody>" +
    Object.keys(api).sort().map(a => `<tr><td><code>${esc(a)}</code></td><td>${api[a].map(chipLink).join(" ")}</td></tr>`).join("") + "</tbody>";
  const fails = Object.entries(N).flatMap(([m, n]) => n.fails.map(f => [m, f]));
  $("#fails").innerHTML = "<thead><tr><th>Function</th><th>Fail(…) message (the attempt goes to Recover)</th></tr></thead><tbody>" +
    fails.map(([m, f]) => `<tr><td>${chipLink(m)}</td><td>${esc(f)}</td></tr>`).join("") + "</tbody>";
  $("#files").innerHTML = "<thead><tr><th>File</th><th>Lines</th><th>Holds</th></tr></thead><tbody>" +
    D.files.map(f => `<tr><td><code>src/${esc(f.file)}</code></td><td>${f.lines}</td><td>${esc(f.about)}</td></tr>`).join("") + "</tbody>";
}

document.addEventListener("click", e => {
  const go = e.target.closest("[data-go]");
  if (go) { e.preventDefault(); select(go.dataset.go); chipEls[go.dataset.go]?.scrollIntoView({block: "nearest"}); }
  if (e.target.id === "clear") { e.preventDefault(); clearSel(); }
});
$("#q").addEventListener("input", search);
$("#hideDiag").addEventListener("change", e => {
  document.querySelector('.col[data-folder="Diagnostics"]').style.display = e.target.checked ? "none" : "";
  drawEdges();
});
$("#theme").addEventListener("click", () => {
  const dark = document.documentElement.dataset.theme ? document.documentElement.dataset.theme === "dark" : matchMedia("(prefers-color-scheme: dark)").matches;
  document.documentElement.dataset.theme = dark ? "light" : "dark";
});
window.addEventListener("resize", drawEdges);
buildMap(); tables();
</script>
</body>
</html>
"""

if __name__ == "__main__":
    sys.exit(main())
