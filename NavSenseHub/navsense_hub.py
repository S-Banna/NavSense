#!/usr/bin/env python3
"""
NavSense Hub (v0.1): a small local web app that ties the stage scripts together.

  py navsense_hub.py                 # opens http://127.0.0.1:8770
  py navsense_hub.py --port 9000 --no-browser

It is deliberately thin. It never imports torch/ROS and holds no pipeline logic: each button runs ONE existing
script as a subprocess, shows its output live, and re-reads manifest.json to show state.

  step 1  Manifest      generate_manifest.py --session <s>
  step 2  Calibration   install_calibration.py <s>          (runs step 1 first if there is no manifest)
  step 3  Depth (DA3)   da3_session_runner.py --session <s> [--dry-run]    (uses the DA3 env's python from hub_config.json)
  --      Doctor        doctor.py --out capabilities.json   (what can this machine run?)
  --      ORB-SLAM3     PLACEHOLDER (teammate). See build_orbslam3_steps() below.

Only one job runs at a time (the GPU, and simpler logs). The server listens on 127.0.0.1 only, and can only run the
fixed actions above on folders that exist directly inside the sessions directory.

Paths (all overridable in hub_config.json under "hub": sessions_dir, generate_manifest, install_calibration, doctor, port):
  sessions   <repo>/NavSenseLogger/sessions
  scripts    searched in NavSenseHub/, NavSenseLogger/, the repo root, and the current directory
  DA3        da3.run_script and da3.python from hub_config.json
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
IMG_EXTS = {".jpg", ".jpeg", ".png"}


# ====================================================================================== config
def read_json(path: Path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except Exception:  # noqa: BLE001
        return None


def find_script(name: str, explicit: str | None) -> Path:
    if explicit:
        return Path(explicit)
    dirs = [HERE, REPO / "NavSenseLogger", REPO, Path.cwd()]
    for d in dirs:
        if (d / name).is_file():
            return d / name
    return dirs[0] / name          # not found: reported in the UI


class Config:
    def __init__(self, cfg_path: Path | None, sessions_override: str | None):
        self.cfg_path = cfg_path if cfg_path and cfg_path.is_file() else None
        raw = read_json(self.cfg_path) if self.cfg_path else None
        raw = raw if isinstance(raw, dict) else {}
        hub = raw.get("hub", {}) if isinstance(raw.get("hub"), dict) else {}
        da3 = raw.get("da3", {}) if isinstance(raw.get("da3"), dict) else {}
        self.port = int(hub.get("port", 8770))
        sess = sessions_override or hub.get("sessions_dir") or str(REPO / "NavSenseLogger" / "sessions")
        if not Path(sess).is_dir() and (Path.cwd() / "sessions").is_dir():
            sess = str(Path.cwd() / "sessions")
        self.sessions = Path(sess).resolve()
        self.generate_manifest = find_script("generate_manifest.py", hub.get("generate_manifest"))
        self.install_calibration = find_script("install_calibration.py", hub.get("install_calibration"))
        self.doctor = find_script("doctor.py", hub.get("doctor"))
        self.da3_script = find_script("da3_session_runner.py", da3.get("run_script") or None)
        self.da3_python = da3.get("python") or sys.executable
        self.caps = HERE / "capabilities.json"

    def describe(self):
        def item(p):
            return {"path": str(p), "exists": Path(p).exists()}
        return {
            "config_file": str(self.cfg_path) if self.cfg_path else None,
            "sessions": item(self.sessions),
            "generate_manifest": item(self.generate_manifest),
            "install_calibration": item(self.install_calibration),
            "doctor": item(self.doctor),
            "da3_script": item(self.da3_script),
            "da3_python": item(self.da3_python) if self.da3_python != sys.executable else {"path": self.da3_python, "exists": True},
        }


# ====================================================================================== sessions
def count_files(d: Path, exts: set[str]) -> int:
    n = 0
    try:
        with os.scandir(d) as it:
            for e in it:
                if e.is_file() and os.path.splitext(e.name)[1].lower() in exts:
                    n += 1
    except OSError:
        pass
    return n


def session_info(sdir: Path) -> dict:
    m = read_json(sdir / "manifest.json") if (sdir / "manifest.json").is_file() else None
    m = m if isinstance(m, dict) else None
    frames = count_files(sdir / "mav0" / "cam0" / "data", IMG_EXTS)
    depth = count_files(sdir / "depth", {".png"})
    meta = read_json(sdir / "depth" / "depth_meta.json") or {}
    cal_file = (sdir / "calibration" / "camera_info.yaml").is_file()
    streams = (m or {}).get("streams") or {}
    stages = (m or {}).get("stages") or {}
    cam = ((m or {}).get("calibration") or {}).get("camera") or {}
    da3 = stages.get("depth_da3") or {}
    src = str(meta.get("intrinsics_source", ""))
    cal_status = cam.get("status")
    return {
        "id": sdir.name,
        "label": (m or {}).get("label", ""),
        "has_manifest": (sdir / "manifest.json").is_file(),
        "manifest_ok": m is not None,
        "frames": frames,
        "gyro_verdict": (streams.get("imu") or {}).get("gyro_verdict"),
        "cal_file": cal_file,
        "cal_status": cal_status,
        "depth_count": depth,
        "depth_focal": meta.get("focal_px_used"),
        "depth_source": src,
        # depth was made from the guessed intrinsics, but a real calibration is installed now
        "depth_stale": bool(depth > 0 and cal_file and cal_status == "calibrated" and src.startswith("orbslam3_phone.yaml")),
        "prepare": (stages.get("prepare") or {}).get("status"),
        "da3_status": da3.get("status"),
        "da3_error": da3.get("error"),
        "orbslam3_status": (stages.get("orbslam3") or {}).get("status"),
    }


def list_sessions(cfg: Config) -> list[dict]:
    if not cfg.sessions.is_dir():
        return []
    dirs = [p for p in cfg.sessions.iterdir() if p.is_dir() and not p.name.startswith(".")]
    return [session_info(p) for p in sorted(dirs, key=lambda p: p.name, reverse=True)]


def resolve_session(cfg: Config, sid: str) -> Path:
    if not sid or sid != Path(sid).name or sid in (".", ".."):
        raise ValueError("bad session id")
    p = (cfg.sessions / sid).resolve()
    if p.parent != cfg.sessions or not p.is_dir():
        raise ValueError(f"no such session: {sid}")
    return p


def mark_depth_cancelled(sdir: Path):
    """If the user cancels DA3 from the hub the runner cannot update its own manifest entry, so do it here."""
    mp = sdir / "manifest.json"
    m = read_json(mp)
    if not isinstance(m, dict):
        return
    st = (m.get("stages") or {}).get("depth_da3")
    if isinstance(st, dict) and st.get("status") == "running":
        st["status"] = "failed"
        st["error"] = "cancelled from the hub"
        st["finished_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        tmp = mp.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(m, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        os.replace(tmp, mp)


# ====================================================================================== jobs
class Busy(Exception):
    pass


class Jobs:
    """One job at a time. A job = a list of (label, command, cwd) steps run in order; stops at the first failure."""

    def __init__(self):
        self.lock = threading.Lock()
        self.id = 0
        self.status = "idle"            # idle | running | done | failed | cancelled
        self.title = ""
        self.action = ""
        self.session = None
        self.lines: list[str] = []
        self.dropped = 0
        self.progress = None
        self.started = self.finished = None
        self.rc = None
        self.proc = None
        self.cancel_requested = False

    def start(self, title, action, session: Path | None, steps, ok_rcs=(0,)):
        with self.lock:
            if self.status == "running":
                raise Busy(f"a job is already running: {self.title}")
            self.id += 1
            self.status, self.title, self.action = "running", title, action
            self.session = session
            self.lines, self.dropped, self.progress = [], 0, None
            self.started, self.finished, self.rc = time.time(), None, None
            self.cancel_requested = False
        threading.Thread(target=self._worker, args=(steps, ok_rcs), daemon=True).start()

    def log(self, line: str):
        with self.lock:
            self.lines.append(line)
            if len(self.lines) > 6000:
                cut = len(self.lines) - 5000
                del self.lines[:cut]
                self.dropped += cut
        m = re.search(r"Processed (\d+)/(\d+)", line)
        if m:
            self.progress = [int(m.group(1)), int(m.group(2))]

    def _worker(self, steps, ok_rcs):
        rc = 0
        env = dict(os.environ, PYTHONUNBUFFERED="1", PYTHONIOENCODING="utf-8")
        for label, cmd, cwd in steps:
            self.log(f"\n=== {label}")
            self.log("$ " + " ".join(f'"{c}"' if " " in c else c for c in cmd))
            try:
                proc = subprocess.Popen(cmd, cwd=str(cwd), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        text=True, encoding="utf-8", errors="replace", bufsize=1, env=env,
                                        stdin=subprocess.DEVNULL)
            except Exception as e:  # noqa: BLE001
                self.log(f"could not start: {e}")
                rc = -1
                break
            self.proc = proc
            for line in proc.stdout:
                self.log(line.rstrip("\r\n"))
            rc = proc.wait()
            self.proc = None
            if self.cancel_requested or rc not in ok_rcs:
                break
        with self.lock:
            self.rc = rc
            self.finished = time.time()
            self.status = "cancelled" if self.cancel_requested else ("done" if rc in ok_rcs else "failed")
        self.log(f"\n=== {self.status} (exit code {rc})")
        if self.cancel_requested and self.action in ("da3", "da3_dry") and self.session:
            try:
                mark_depth_cancelled(self.session)
            except Exception as e:  # noqa: BLE001
                self.log(f"could not mark the manifest cancelled: {e}")

    def cancel(self):
        self.cancel_requested = True
        p = self.proc
        if p and p.poll() is None:
            if os.name == "nt":
                subprocess.run(["taskkill", "/PID", str(p.pid), "/T", "/F"], capture_output=True)
            else:
                p.terminate()

    def snapshot(self, start=0):
        with self.lock:
            base = self.dropped
            idx = max(0, start - base)
            return {"id": self.id, "status": self.status, "title": self.title, "action": self.action,
                    "session": self.session.name if self.session else None,
                    "lines": self.lines[idx:], "next": base + len(self.lines), "progress": self.progress,
                    "started": self.started, "finished": self.finished, "rc": self.rc}


# ====================================================================================== actions
ORBSLAM3_IMPLEMENTED = False


def build_orbslam3_steps(cfg: Config, session: Path):
    """
    ===== PLACEHOLDER (teammate): ORB-SLAM3 =====
    NOT IMPLEMENTED ON PURPOSE. To wire it in:
      1. Write the stage script per SESSION_CONTRACT.md section 5 ("stage runner interface") and section 8:
            <orbslam3 command> --session <folder> [--params <json>] [--dry-run]
         It must update stages.orbslam3 in manifest.json and write slam/orbslam3/trajectory_tum.txt + trajectory_meta.json.
      2. Return the steps here, e.g.
            return [("ORB-SLAM3", [python, str(script), "--session", str(session)], script.parent)]
      3. Set ORBSLAM3_IMPLEMENTED = True (the UI button enables itself from that flag).
      4. Fill the "orbslam3" block of hub_config.json so doctor.py can verify the install.
    """
    raise NotImplementedError("ORB-SLAM3 is a placeholder: see build_orbslam3_steps() in navsense_hub.py")


def start_action(cfg: Config, jobs: Jobs, action: str, sid: str | None):
    py = sys.executable
    if action == "doctor":
        cmd = [py, str(cfg.doctor), "--out", str(cfg.caps), "--no-color", "--verbose"]
        if cfg.cfg_path:
            cmd += ["--config", str(cfg.cfg_path)]
        jobs.start("Doctor (machine readiness)", action, None, [("doctor", cmd, HERE)], ok_rcs=(0, 1))   # rc 1 = some FAIL
        return
    s = resolve_session(cfg, sid or "")
    gen = ("1. Generate manifest", [py, str(cfg.generate_manifest), "--session", str(s)], cfg.generate_manifest.parent)
    for p, what in ((cfg.generate_manifest, "generate_manifest.py"),):
        if action == "manifest" and not p.is_file():
            raise ValueError(f"{what} not found at {p}")
    if action == "manifest":
        jobs.start(f"Manifest: {s.name}", action, s, [gen])
    elif action == "calibration":
        if not cfg.install_calibration.is_file():
            raise ValueError(f"install_calibration.py not found at {cfg.install_calibration}")
        steps = ([gen] if not (s / "manifest.json").is_file() else [])
        steps.append(("2. Install calibration", [py, str(cfg.install_calibration), str(s)], cfg.install_calibration.parent))
        jobs.start(f"Calibration: {s.name}", action, s, steps)
    elif action in ("da3", "da3_dry"):
        if not cfg.da3_script.is_file():
            raise ValueError(f"DA3 runner not found at {cfg.da3_script} (set da3.run_script in hub_config.json)")
        if not (s / "manifest.json").is_file():
            raise ValueError("no manifest.json: run step 1 (Manifest) first. The DA3 stage only reads the manifest and calibration.")
        cmd = [cfg.da3_python, str(cfg.da3_script), "--session", str(s)] + (["--dry-run"] if action == "da3_dry" else [])
        jobs.start(f"DA3{' dry run' if action == 'da3_dry' else ''}: {s.name}", action, s,
                   [("3. Depth (DA3)", cmd, cfg.da3_script.parent)])
    elif action == "orbslam3":
        if not ORBSLAM3_IMPLEMENTED:
            raise NotImplementedError("ORB-SLAM3 is a placeholder for the teammate (see build_orbslam3_steps in navsense_hub.py)")
        jobs.start(f"ORB-SLAM3: {s.name}", action, s, build_orbslam3_steps(cfg, s))
    elif action in ("open_folder", "open_depth"):
        target = s / "depth" if action == "open_depth" else s
        if not target.is_dir():
            raise ValueError("folder does not exist")
        if os.name == "nt":
            os.startfile(str(target))      # noqa: S606
        elif sys.platform == "darwin":
            subprocess.Popen(["open", str(target)])
        else:
            subprocess.Popen(["xdg-open", str(target)])
    else:
        raise ValueError(f"unknown action: {action}")


# ====================================================================================== http
def make_handler(cfg: Config, jobs: Jobs):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            if u.path == "/":
                body = PAGE.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif u.path == "/api/state":
                caps = read_json(cfg.caps)
                self._json({"config": cfg.describe(), "sessions": list_sessions(cfg), "doctor": caps,
                            "doctor_age_s": (time.time() - cfg.caps.stat().st_mtime) if cfg.caps.is_file() else None,
                            "orbslam3_implemented": ORBSLAM3_IMPLEMENTED})
            elif u.path == "/api/job":
                start = int(parse_qs(u.query).get("from", ["0"])[0] or 0)
                self._json(jobs.snapshot(start))
            else:
                self.send_error(404)

        def do_POST(self):
            if self.headers.get("X-NavSense") != "1":          # blocks cross-site form posts
                return self._json({"error": "forbidden"}, 403)
            try:
                n = int(self.headers.get("Content-Length", 0))
                data = json.loads(self.rfile.read(n) or b"{}")
            except Exception:  # noqa: BLE001
                return self._json({"error": "bad request"}, 400)
            u = urlparse(self.path)
            try:
                if u.path == "/api/run":
                    start_action(cfg, jobs, str(data.get("action", "")), data.get("session"))
                    return self._json({"ok": True})
                if u.path == "/api/cancel":
                    jobs.cancel()
                    return self._json({"ok": True})
            except Busy as e:
                return self._json({"error": str(e)}, 409)
            except NotImplementedError as e:
                return self._json({"error": str(e)}, 501)
            except Exception as e:  # noqa: BLE001
                return self._json({"error": str(e)}, 400)
            self.send_error(404)

    return H


# ====================================================================================== page
PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>NavSense Hub</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
:root{--bg:#f6f7f9;--fg:#1b1f24;--mut:#6a737d;--card:#fff;--bd:#d8dde3;--ok:#1a7f37;--okb:#dafbe1;--warn:#9a6700;--warnb:#fff8c5;--bad:#cf222e;--badb:#ffebe9;--acc:#0b5fff;--mono:ui-monospace,Consolas,monospace}
@media (prefers-color-scheme:dark){:root{--bg:#0f1318;--fg:#e6e9ed;--mut:#8b949e;--card:#171c23;--bd:#2b323b;--ok:#56d364;--okb:#12261a;--warn:#e3b341;--warnb:#2b2410;--bad:#ff7b72;--badb:#2e1517;--acc:#6ea8ff}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.45 system-ui,Segoe UI,sans-serif}
header{display:flex;align-items:center;gap:12px;padding:12px 20px;border-bottom:1px solid var(--bd);background:var(--card);flex-wrap:wrap}
h1{font-size:17px;margin:0 8px 0 0}main{padding:16px 20px;max-width:1400px;margin:auto}
.card{background:var(--card);border:1px solid var(--bd);border-radius:8px;padding:12px 14px;margin-bottom:14px}
h2{font-size:14px;margin:0 0 8px}.mut{color:var(--mut)}.small{font-size:12px}
button{font:inherit;padding:4px 10px;border:1px solid var(--bd);border-radius:6px;background:var(--card);color:var(--fg);cursor:pointer}
button:hover:not(:disabled){border-color:var(--acc);color:var(--acc)}button:disabled{opacity:.45;cursor:not-allowed}
button.primary{background:var(--acc);border-color:var(--acc);color:#fff}button.primary:hover:not(:disabled){color:#fff;opacity:.9}
.chip{display:inline-block;padding:1px 8px;border-radius:10px;font-size:12px;border:1px solid var(--bd);white-space:nowrap}
.ok{color:var(--ok);background:var(--okb);border-color:transparent}.warn{color:var(--warn);background:var(--warnb);border-color:transparent}
.bad{color:var(--bad);background:var(--badb);border-color:transparent}.mute{color:var(--mut)}
table{width:100%;border-collapse:collapse}th,td{text-align:left;padding:8px 8px;border-bottom:1px solid var(--bd);vertical-align:top}
th{font-size:12px;color:var(--mut);font-weight:600}td .row{display:flex;gap:6px;margin-top:6px;flex-wrap:wrap}
.sid{font-family:var(--mono);font-weight:600}.stages{display:flex;gap:8px;flex-wrap:wrap}
pre{margin:0;background:#0b0e12;color:#d7dde4;padding:10px;border-radius:6px;max-height:340px;overflow:auto;font:12px/1.4 var(--mono);white-space:pre-wrap}
.bar{height:8px;background:var(--bd);border-radius:4px;overflow:hidden;margin:8px 0}.bar>div{height:100%;background:var(--acc);width:0}
.ph{border:1px dashed var(--warn)}.ph h2{color:var(--warn)}a{color:var(--acc)}
</style></head><body>
<header><h1>NavSense Hub</h1>
<button id="doctorBtn">Run doctor</button><button id="refreshBtn">Refresh</button>
<span id="doctorAge" class="mut small"></span><span style="flex:1"></span><span id="busy" class="chip warn" hidden></span></header>
<main>
<div class="card"><h2>This machine (doctor)</h2><div id="stages" class="stages"></div><div id="cfgwarn" class="small" style="margin-top:8px"></div></div>
<div class="card"><h2>Sessions <span class="mut small" id="sessDir"></span></h2>
<table><thead><tr><th>Session</th><th>Recorded</th><th>1 &middot; Manifest</th><th>2 &middot; Calibration</th><th>3 &middot; Depth (DA3)</th><th>ORB-SLAM3</th></tr></thead><tbody id="rows"></tbody></table>
<div id="empty" class="mut" hidden>No session folders found.</div></div>
<div class="card"><h2>Job output <span id="jobTitle" class="mut"></span> <button id="cancelBtn" hidden>Cancel</button></h2>
<div class="bar"><div id="prog"></div></div><pre id="log">Nothing run yet.</pre></div>
<div class="card ph"><h2>ORB-SLAM3 &mdash; PLACEHOLDER (left for the teammate)</h2>
<div class="small">The hub intentionally does <b>not</b> run ORB-SLAM3 yet. Its button above is disabled until it is wired in.
What to do: read <code>SESSION_CONTRACT.md</code> section 8 and the <code>orbslam3</code> stage's <code>todo</code> list in any session's <code>manifest.json</code>;
write the stage script (<code>--session &lt;folder&gt; [--dry-run]</code>, results in <code>slam/orbslam3/</code>); then implement
<code>build_orbslam3_steps()</code> in <code>navsense_hub.py</code>, set <code>ORBSLAM3_IMPLEMENTED = True</code>, and fill the <code>orbslam3</code> block in <code>hub_config.json</code>.
Machine-side inputs already exist per session: <code>mav0/</code>, <code>timestamps.txt</code>, <code>orbslam3_phone.yaml</code> (its intrinsics are still a guess; the real ones are in <code>calibration/camera_info.yaml</code>).</div></div>
<div class="small mut">Not wired into the hub yet: RTAB-Map (WSL2), live ingest, Nav2, VLM. Use the commands in PROJECT_SUMMARY.md for those.</div>
</main>
<script>
const $=s=>document.querySelector(s);
const esc=t=>String(t==null?'':t).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
let state=null,jobNext=0,jobId=0,lastStatus='idle',busy=false;
async function api(path,body){const o=body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json','X-NavSense':'1'},body:JSON.stringify(body)};const r=await fetch(path,o);return r.json();}
const chip=(cls,t,title)=>`<span class="chip ${cls}"${title?` title="${esc(title)}"`:''}>${esc(t)}</span>`;
function btn(label,action,sid,{dis=false,why='',primary=false}={}){
  return `<button ${primary?'class="primary"':''} ${dis||busy?'disabled':''} title="${esc(why)}" data-a="${action}" data-s="${esc(sid)}">${esc(label)}</button>`;}
function render(){
  if(!state)return;const c=state.config;
  $('#sessDir').textContent=c.sessions.path;
  const warn=[];for(const [k,v] of Object.entries(c)){if(v&&typeof v==='object'&&v.exists===false)warn.push(`${k}: not found (${v.path})`);}
  $('#cfgwarn').innerHTML=warn.length?chip('bad','config problem')+' '+esc(warn.join(' | ')):'';
  const d=state.doctor,st=$('#stages');st.innerHTML='';
  const names=['prepare','depth_da3','rtabmap_offline','orbslam3'];
  if(!d){st.innerHTML='<span class="mut">Doctor has not been run yet. Click "Run doctor".</span>';}
  else for(const n of names){const s=(d.stages||{})[n];let h;
    if(n==='orbslam3')h=chip('warn','orbslam3: placeholder (teammate)',(s&&s.blocked_by||[]).join('\n'));
    else if(!s||s.ready===null)h=chip('mute',n+': not checked');
    else h=chip(s.ready?'ok':'bad',n+(s.ready?': ready':': not ready'),(s.blocked_by||[]).join('\n'));
    st.insertAdjacentHTML('beforeend',h);}
  $('#doctorAge').textContent=state.doctor_age_s!=null?`doctor ran ${Math.round(state.doctor_age_s/60)} min ago`:'';
  const rows=$('#rows');rows.innerHTML='';$('#empty').hidden=state.sessions.length>0;
  const da3ok=c.da3_script.exists;
  for(const s of state.sessions){
    const hasF=s.frames>0;
    const manifest=s.has_manifest?(s.manifest_ok?chip('ok','manifest'):chip('bad','manifest unreadable')):chip('bad','no manifest');
    let cal;
    if(s.cal_file&&s.cal_status==='calibrated')cal=chip('ok','calibrated');
    else if(s.cal_file)cal=chip('warn','file installed, manifest says '+(s.cal_status||'none'));
    else cal=chip('warn','not installed','DA3 would fall back to the guessed intrinsics in orbslam3_phone.yaml');
    let dep;
    if(s.depth_count===0)dep=chip('mute','no depth');
    else if(s.depth_count<s.frames)dep=chip('warn',`partial ${s.depth_count}/${s.frames}`);
    else dep=chip('ok',`${s.depth_count}/${s.frames} depth`);
    if(s.depth_focal)dep+=' '+chip('mute','focal '+Number(s.depth_focal).toFixed(1));
    if(s.depth_stale)dep+=' '+chip('warn','made with guessed focal: rerun');
    if(s.da3_status==='running'&&!(state_job_running(s)))dep+=' '+chip('warn','stage says running (stale?)');
    if(s.da3_status==='failed')dep+=' '+chip('bad','last run failed',s.da3_error||'');
    const gyro=s.gyro_verdict?chip(s.gyro_verdict==='ok'?'ok':'bad','gyro '+s.gyro_verdict):'';
    const dwhy=!s.has_manifest?'Generate the manifest first':(!hasF?'no frames':(!da3ok?'DA3 runner not found':''));
    const orb=state.orbslam3_implemented?btn('Run','orbslam3',s.id,{dis:!s.has_manifest}):btn('Run','orbslam3',s.id,{dis:true,why:'Placeholder: left for the teammate'});
    rows.insertAdjacentHTML('beforeend',`<tr>
      <td><span class="sid">${esc(s.id)}</span><div class="mut small">${esc(s.label)}</div><div class="row">${btn('Open folder','open_folder',s.id)}</div></td>
      <td>${s.frames} frames<div style="margin-top:4px">${gyro}</div></td>
      <td>${manifest}<div class="row">${btn(s.has_manifest?'Regenerate':'Generate','manifest',s.id,{primary:!s.has_manifest})}</div></td>
      <td>${cal}<div class="row">${btn('Install calibration','calibration',s.id,{dis:!hasF,why:hasF?'':'no frames'})}</div></td>
      <td>${dep}<div class="row">${btn('Dry run','da3_dry',s.id,{dis:!!dwhy,why:dwhy})}${btn(s.depth_count?'Rerun DA3':'Run DA3','da3',s.id,{dis:!!dwhy,why:dwhy,primary:s.has_manifest&&s.depth_count===0})}${s.depth_count?btn('Open depth','open_depth',s.id):''}</div></td>
      <td>${chip('warn','placeholder')}<div class="row">${orb}</div></td></tr>`);}
}
function state_job_running(s){return busy&&window._jobSession===s.id;}
async function refresh(){state=await api('/api/state');render();}
document.addEventListener('click',async e=>{
  const b=e.target.closest('button[data-a]');if(!b)return;
  const a=b.dataset.a,sid=b.dataset.s,s=(state.sessions||[]).find(x=>x.id===sid);
  if((a==='da3')&&s){
    const d=state.doctor&&state.doctor.stages&&state.doctor.stages.depth_da3;
    if(d&&d.ready===false&&!confirm('Doctor says DA3 is not ready on this machine:\n'+d.blocked_by.join('\n')+'\n\nRun anyway?'))return;
    if(!(s.cal_file&&s.cal_status==='calibrated')&&!confirm('This session has no installed calibration: DA3 will use the GUESSED focal length from orbslam3_phone.yaml, so metric depth will be wrong.\n\nRun anyway?'))return;
    if(s.depth_count>0&&!confirm('Depth already exists and will be overwritten. Continue?'))return;}
  const r=await api('/api/run',{action:a,session:sid});
  if(r.error)alert(r.error);else{jobNext=0;$('#log').textContent='';window._jobSession=sid;}
  pollJob();});
$('#doctorBtn').onclick=async()=>{const r=await api('/api/run',{action:'doctor'});if(r.error)alert(r.error);else{jobNext=0;$('#log').textContent='';}pollJob();};
$('#refreshBtn').onclick=refresh;$('#cancelBtn').onclick=()=>api('/api/cancel',{});
async function pollJob(){
  const j=await api('/api/job?from='+jobNext);
  if(j.id!==jobId){jobId=j.id;if(jobNext===0)$('#log').textContent='';}
  if(j.lines.length){const el=$('#log');const atEnd=el.scrollTop+el.clientHeight>=el.scrollHeight-20;
    el.textContent+=(el.textContent?'\n':'')+j.lines.join('\n');if(atEnd)el.scrollTop=el.scrollHeight;}
  jobNext=j.next;
  const nb=j.status==='running';
  if(nb!==busy){busy=nb;render();}
  $('#busy').hidden=!nb;$('#busy').textContent='running: '+j.title;
  $('#cancelBtn').hidden=!nb;$('#jobTitle').textContent=j.title?(' - '+j.title+' ['+j.status+']'):'';
  $('#prog').style.width=j.progress?(100*j.progress[0]/Math.max(1,j.progress[1]))+'%':(nb?'0':(j.status==='done'?'100%':'0'));
  if(lastStatus==='running'&&!nb)await refresh();
  lastStatus=j.status;}
refresh().then(pollJob);setInterval(pollJob,1000);
</script></body></html>
"""


def main():
    ap = argparse.ArgumentParser(description="NavSense Hub")
    ap.add_argument("--config", type=Path, default=HERE / "hub_config.json")
    ap.add_argument("--sessions", help="sessions folder (default: <repo>/NavSenseLogger/sessions)")
    ap.add_argument("--port", type=int)
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()
    cfg = Config(a.config, a.sessions)
    port = a.port or cfg.port
    print("NavSense Hub")
    for k, v in cfg.describe().items():
        flag = "" if (v is None or isinstance(v, str) or v.get("exists")) else "   <-- NOT FOUND"
        print(f"  {k:20s} {v if isinstance(v, (str, type(None))) else v['path']}{flag}")
    jobs = Jobs()
    server = ThreadingHTTPServer(("127.0.0.1", port), make_handler(cfg, jobs))
    url = f"http://127.0.0.1:{port}"
    print(f"\nListening on {url}   (Ctrl+C to stop)")
    if not a.no_browser:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping")
        jobs.cancel()


if __name__ == "__main__":
    main()