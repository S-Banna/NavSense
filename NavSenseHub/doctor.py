#!/usr/bin/env python3
"""
NavSense doctor: is THIS machine ready for each NavSense stage?

  python doctor.py                      # human-readable report
  python doctor.py --json               # machine-readable (what the hub reads)
  python doctor.py --out capabilities.json
  python doctor.py --only da3,ros2      # subset of: da3, ros2, orbslam3, vlm
  python doctor.py --smoke              # also run the configured smoke commands (slow)
  python doctor.py --init-config        # write hub_config.json from the defaults and exit

Where to run it
  * Windows (PowerShell): checks DA3 natively and reaches into WSL for ROS 2 / RTAB-Map. This is the main case.
  * WSL / Linux: checks ROS 2 natively; the DA3 path is checked but the DA3 Python probe needs Windows.
  * A machine that has only ORB-SLAM3: set da3.enabled and ros2.enabled to false in hub_config.json.

Every check is one of: ok / warn / fail / todo / skip. A STAGE is READY when none of the checks that gate it is
fail / todo / skip. (warn never blocks.) Exit code: 0 = no FAIL anywhere, 1 = at least one FAIL, 2 = doctor crashed.
"""
from __future__ import annotations

import argparse
import base64
import copy
import json
from pathlib import Path
import os
import platform
import re
import shutil
import socket
import subprocess
import sys
import time
from dataclasses import asdict, dataclass, field

OK, WARN, FAIL, TODO, SKIP = "ok", "warn", "fail", "todo", "skip"
STAGES = ["prepare", "depth_da3", "rtabmap_offline", "orbslam3", "live_ingest", "rtabmap_live", "nav2", "vlm"]
IS_WINDOWS = os.name == "nt"


def _is_wsl():
    try:
        return "microsoft" in open("/proc/version").read().lower()
    except OSError:
        return False


IS_WSL = _is_wsl()

DEFAULT_CONFIG = {
    "phone_port": 8765,
    "da3": {
        "enabled": True,
        "path": r"C:\FYP\Depth-Anything-3",
        "python": "",            
        "conda_env": "",         
        "model": "",             
        "checkpoint_path": "",   
        "require_cuda": True,
        "run_script": "",        
        "smoke_command": [],     
    },
    "ros2": {
        "enabled": True,
        "wsl_distro": "",        
        "setup": "/opt/ros/jazzy/setup.bash",
        "workspace": "$HOME/navsense_ws",
        "maps_dir": "$HOME/maps",
        "check_nav2": True,
    },
    "orbslam3": {
        "enabled": False,        
        "required_paths": [],    
        "smoke_command": [],     
        "smoke_expect_rc": [0],
        "notes": "",
    },
    "vlm": {
        "enabled": True,
        "python": "",            # Full path to python.exe with open_clip. "" = sys.executable
        "script_path": "",       # Path to vlm_openclip.py. "" = look next to doctor.py
        "smoke_command": [],
        "smoke_expect_rc": [0],
        "notes": "",
    },
}


# ----------------------------------------------------------------------------------------------- plumbing
@dataclass
class Check:
    group: str
    name: str
    status: str
    detail: str = ""
    fix: str = ""
    gates: list = field(default_factory=list)


class Report:
    def __init__(self):
        self.checks: list[Check] = []
        self.covered: set[str] = set()

    def cover(self, *stages):
        self.covered.update(stages)

    def add(self, group, name, status, detail="", fix="", gates=()):
        self.checks.append(Check(group, name, status, detail, fix, list(gates)))

    def stage_state(self):
        out = {}
        for st in STAGES:
            if st not in self.covered:
                out[st] = {"ready": None, "blocked_by": ["not checked (see --only)"]}
                continue
            reasons = [f"{c.group}: {c.name} [{c.status}]" for c in self.checks
                       if st in c.gates and c.status in (FAIL, TODO, SKIP)]
            out[st] = {"ready": not reasons, "blocked_by": reasons}
        return out


def deep_merge(base, extra):
    for k, v in extra.items():
        if k.startswith("_"):
            continue
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            deep_merge(base[k], v)
        else:
            base[k] = v
    return base


def load_config(path):
    cfg = copy.deepcopy(DEFAULT_CONFIG)
    note = "no hub_config.json found: using defaults"
    if path and os.path.isfile(path):
        with open(path, encoding="utf-8") as f:
            deep_merge(cfg, json.load(f))
        note = f"config: {os.path.abspath(path)}"
    return cfg, note


def run(cmd, timeout=20):
    """Never raises. Returns (rc, stdout, stderr). 124 = timeout, 127 = not found."""
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, encoding="utf-8", errors="replace")
        return p.returncode, p.stdout.strip(), p.stderr.strip()
    except FileNotFoundError:
        return 127, "", f"not found: {cmd[0]}"
    except subprocess.TimeoutExpired:
        return 124, "", f"timed out after {timeout}s"
    except Exception as e:  # noqa: BLE001
        return 1, "", str(e)


def to_local_path(p):
    """Translate C:\\x\\y to /mnt/c/x/y when running inside WSL."""
    m = re.match(r"^([A-Za-z]):[\\/](.*)$", p or "")
    if m and not IS_WINDOWS:
        return f"/mnt/{m.group(1).lower()}/" + m.group(2).replace("\\", "/")
    return p


def looks_windows_path(p):
    return bool(re.match(r"^[A-Za-z]:[\\/]", p or ""))


def bash_in_ros_env(script, cfg, timeout=90):
    """Run a bash script where ROS 2 lives (WSL from Windows, or this machine on Linux/WSL)."""
    if IS_WINDOWS:
        cmd = ["wsl.exe"] + (["-d", cfg["ros2"]["wsl_distro"]] if cfg["ros2"]["wsl_distro"] else []) + \
              ["-e", "bash", "-lc", script]
    else:
        cmd = ["bash", "-lc", script]
    return run(cmd, timeout)


# ----------------------------------------------------------------------------------------------- DA3
DA3_PROBE = r'''
import json, os, sys, importlib, shutil, sysconfig
ROOT = __ROOT__
out = {"python": sys.version.split()[0], "exe": sys.executable}
def ver(m):
    try:
        mod = importlib.import_module(m)
        return str(getattr(mod, "__version__", "ok"))
    except Exception:
        return None
for m in ("numpy", "cv2", "PIL", "huggingface_hub", "safetensors", "xformers", "einops"):
    out[m] = ver(m)
try:
    import torch
    out["torch"] = torch.__version__
    out["cuda_build"] = torch.version.cuda
    out["cuda"] = bool(torch.cuda.is_available())
    if out["cuda"]:
        p = torch.cuda.get_device_properties(0)
        out["gpu"] = p.name
        out["vram_gb"] = round(p.total_memory / 2**30, 1)
except Exception as e:
    out["torch"] = None
    out["torch_err"] = str(e)[:200]
out["da3"], out["da3_via"] = None, None

for mode in ("installed", "source"):
    try:
        if mode == "source":
            src = os.path.join(ROOT, "src")
            if os.path.isdir(src):
                sys.path.insert(0, src)

        for name in list(sys.modules):
            if name == "depth_anything_3" or name.startswith("depth_anything_3."):
                sys.modules.pop(name, None)

        importlib.invalidate_caches()
        api = importlib.import_module("depth_anything_3.api")

        if not hasattr(api, "DepthAnything3"):
            raise ImportError(
                "depth_anything_3.api imported, but DepthAnything3 was not found"
            )

        out["da3"] = str(getattr(api, "__version__", "imported"))
        out["da3_file"] = str(getattr(api, "__file__", ""))
        out["da3_via"] = mode
        break

    except Exception as e:
        out["da3_err"] = str(e)[:300]
scripts = sysconfig.get_path("scripts") or ""
out["da3_cli"] = bool(shutil.which("da3", path=scripts + os.pathsep + os.environ.get("PATH", "")))
home = os.environ.get("HF_HOME") or os.path.join(os.path.expanduser("~"), ".cache", "huggingface")
hub = os.path.join(home, "hub")
out["hf_cache"] = hub
out["hf_models"] = sorted(n for n in (os.listdir(hub) if os.path.isdir(hub) else []) if "da3" in n.lower())
print("@@JSON@@" + json.dumps(out))
'''


def find_da3_python(c, lroot):
    cands = []
    if c.get("python"):
        cands.append(([c["python"]], "configured da3.python"))
    for rel in (".venv/Scripts/python.exe", "venv/Scripts/python.exe", ".venv/bin/python", "venv/bin/python"):
        p = os.path.join(lroot, *rel.split("/"))
        if os.path.isfile(p):
            cands.append(([p], "virtualenv inside the DA3 folder"))
    if c.get("conda_env"):
        cands.append((["conda", "run", "--no-capture-output", "-n", c["conda_env"], "python"], f"conda env {c['conda_env']}"))
    cands.append(([sys.executable], "the interpreter running doctor.py (activate your DA3 env first)"))
    cands.append((["python"], "python on PATH"))
    for cmd, why in cands:
        rc, out, _ = run(cmd + ["-c", "import sys;print(sys.executable)"], 30)
        if rc == 0 and out:
            return cmd, why, out.splitlines()[-1]
    return None, "", ""


def check_da3(cfg, rep, smoke):
    c, G, gate = cfg["da3"], "DA3", ["depth_da3"]
    rep.cover("depth_da3")
    if not c.get("enabled", True):
        rep.add(G, "DA3", SKIP, "disabled in hub_config.json on this machine", gates=gate)
        return
    root = c["path"]
    lroot = to_local_path(root)
    if not os.path.isdir(lroot):
        rep.add(G, "DA3 folder", FAIL, f"not found: {root}",
                fix="git clone https://github.com/ByteDance-Seed/Depth-Anything-3 there, or fix da3.path in hub_config.json",
                gates=gate)
        return
    rep.add(G, "DA3 folder", OK, root)
    has_src = os.path.isdir(os.path.join(lroot, "src", "depth_anything_3"))
    rep.add(G, "DA3 source tree", OK if has_src else WARN,
            "src/depth_anything_3 found" if has_src else "src/depth_anything_3 not found: is da3.path the repo root?")

    if not IS_WINDOWS and looks_windows_path(root):
        rep.add(G, "DA3 python probe", WARN,
                "running inside WSL: cannot probe the Windows Python environment from here",
                fix="run doctor.py from Windows PowerShell (inside your DA3 env) for the full DA3 check", gates=gate)
    else:
        cmd, why, exe = find_da3_python(c, lroot)
        if cmd is None:
            rep.add(G, "DA3 python", FAIL, "no working Python found", fix="set da3.python (or da3.conda_env) in hub_config.json",
                    gates=gate)
        else:
            rep.add(G, "DA3 python", OK, f"{exe}  (found via: {why})")
            code = DA3_PROBE.replace("__ROOT__", json.dumps(root))
            wrapper = "import base64;exec(base64.b64decode('%s').decode())" % base64.b64encode(code.encode()).decode()
            t0 = time.time()
            rc, out, err = run(cmd + ["-c", wrapper], 180)
            line = next((l for l in out.splitlines() if l.startswith("@@JSON@@")), None)
            if line is None:
                rep.add(G, "DA3 probe", FAIL, f"probe failed (rc {rc}): {(err or out)[-300:]}", gates=gate)
            else:
                evaluate_da3_probe(json.loads(line[8:]), c, rep, G, gate, time.time() - t0)

    rs = c.get("run_script", "")
    if not rs:
        rep.add(G, "DA3 runner script", TODO,
                "da3.run_script not set: the hub's 'run DA3' button needs a script that takes a session folder",
                fix="set da3.run_script in hub_config.json (see SESSION_CONTRACT.md, stage depth_da3)", gates=gate)
    else:
        p = rs if os.path.isabs(rs) or looks_windows_path(rs) else os.path.join(lroot, rs)
        p = to_local_path(p)
        rep.add(G, "DA3 runner script", OK if os.path.isfile(p) else FAIL, p,
                fix="" if os.path.isfile(p) else "fix da3.run_script", gates=gate)

    sc = c.get("smoke_command") or []
    if not sc:
        rep.add(G, "DA3 smoke test", SKIP, "no da3.smoke_command configured")
    elif not smoke:
        rep.add(G, "DA3 smoke test", SKIP, "configured; run with --smoke")
    else:
        rc, out, err = run(sc, 900)
        rep.add(G, "DA3 smoke test", OK if rc == 0 else FAIL, f"rc {rc}: {(err or out)[-200:]}", gates=gate if rc else [])


def evaluate_da3_probe(i, c, rep, G, gate, secs):
    pv = tuple(int(x) for x in re.findall(r"\d+", i.get("python", "0.0"))[:2] or [0, 0])
    rep.add(G, "Python version", OK if pv >= (3, 9) else WARN, i.get("python", "?"),
            fix="" if pv >= (3, 9) else "DA3 expects a recent Python (3.9+; the app extras want 3.10+)")
    if not i.get("torch"):
        rep.add(G, "PyTorch", FAIL, i.get("torch_err", "not installed"),
                fix="pip install torch torchvision (a CUDA build), then xformers", gates=gate)
    else:
        rep.add(G, "PyTorch", OK, f"{i['torch']} (CUDA build {i.get('cuda_build')})")
        if i.get("cuda"):
            rep.add(G, "CUDA GPU", OK, f"{i.get('gpu')} | {i.get('vram_gb')} GB")
        else:
            req = c.get("require_cuda", True)
            rep.add(G, "CUDA GPU", FAIL if req else WARN, "torch.cuda.is_available() is False: DA3 would run on CPU (far too slow)",
                    fix="install a CUDA build of torch matching your driver, or set da3.require_cuda=false", gates=gate if req else [])
    rep.add(G, "xformers", OK if i.get("xformers") else WARN, i.get("xformers") or "not installed (the DA3 README lists it)")
    if i.get("da3"):
        via = i.get("da3_via")
        rep.add(G, "depth_anything_3 import", OK if via == "installed" else WARN,
                f"{i['da3']} from {i.get('da3_file')}" + ("" if via == "installed" else
                                                         "  (only importable from the source tree: run `pip install -e .` in the DA3 folder)"))
    else:
        rep.add(G, "depth_anything_3 import", FAIL, i.get("da3_err", "import failed"),
                fix="cd the DA3 folder, then: pip install -e .", gates=gate)
    rep.add(G, "da3 CLI", OK if i.get("da3_cli") else WARN, "found" if i.get("da3_cli") else "`da3` command not found in this env")
    cached = i.get("hf_models", [])
    want = c.get("model", "")
    ck = c.get("checkpoint_path", "")
    if ck:
        okp = os.path.exists(to_local_path(ck))
        rep.add(G, "DA3 weights", OK if okp else FAIL, ck, fix="" if okp else "fix da3.checkpoint_path", gates=[] if okp else gate)
    elif want:
        tag = "models--" + want.replace("/", "--")
        hit = any(m.lower() == tag.lower() for m in cached)
        rep.add(G, "DA3 weights", OK if hit else WARN, f"{want} cached" if hit else
                f"{want} not in the Hugging Face cache: the first run downloads it (needs internet)")
    else:
        rep.add(G, "DA3 weights", OK if cached else WARN,
                ", ".join(cached) if cached else "no DA3 weights in the Hugging Face cache yet (first run downloads)")
    rep.add(G, "probe time", OK, f"{secs:.0f} s")


# ----------------------------------------------------------------------------------------------- ROS 2
ROS_PKGS_REQUIRED = ["rtabmap_launch", "rtabmap_slam", "rtabmap_odom", "rtabmap_viz", "navsense_bridge"]
ROS_PKGS_OPTIONAL = ["teleop_twist_keyboard", "camera_calibration", "rqt_image_view", "robot_localization"]
ROS_PKGS_NAV2 = ["nav2_bringup", "nav2_map_server"]

ROS_SCRIPT = r'''
echo "@@distro=$(lsb_release -ds 2>/dev/null)"
echo "@@display=${DISPLAY:-}${WAYLAND_DISPLAY:-}"
if [ -f @SETUP@ ]; then echo "@@setup=1"; source @SETUP@ >/dev/null 2>&1; else echo "@@setup=0"; fi
echo "@@ros_distro=${ROS_DISTRO:-}"
echo "@@ros2=$(command -v ros2)"
if [ -f @WS@/install/setup.bash ]; then echo "@@ws=1"; source @WS@/install/setup.bash >/dev/null 2>&1; else echo "@@ws=0"; fi
for p in @PKGS@; do echo "@@pkg:$p=$(ros2 pkg prefix $p 2>/dev/null | head -1)"; done
echo "@@exe=$(ros2 pkg executables navsense_bridge 2>/dev/null | tr '\n' ',')"
echo "@@pydeps=$(python3 - <<'PY'
import importlib
out = []
for m in ("av", "cv2", "numpy", "yaml"):
    try:
        mod = importlib.import_module(m)
        out.append(m + "=" + str(getattr(mod, "__version__", "ok")))
    except Exception:
        out.append(m + "=-")
print(";".join(out))
PY
)"
timeout 30 ros2 daemon start >/dev/null 2>&1
echo "@@daemon_start_rc=$?"

echo "@@daemon=$(timeout 10 ros2 daemon status 2>&1 | head -1)"

timeout 20 ros2 topic list >/dev/null 2>&1
echo "@@topic_rc=$?"
echo "@@port=$(python3 -c "import socket;s=socket.socket();s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);s.bind(('0.0.0.0',@PORT@));print('free')" 2>&1 | tail -1)"
[ -d @MAPS@ ] && echo "@@maps=1" || echo "@@maps=0"
'''


def parse_facts(out):
    facts = {}
    for line in out.splitlines():
        if line.startswith("@@") and "=" in line:
            k, v = line[2:].split("=", 1)
            facts[k] = v.strip()
    return facts


def check_ros2(cfg, rep):
    c, G = cfg["ros2"], "ROS 2"
    base = ["rtabmap_offline", "live_ingest", "rtabmap_live", "nav2"]
    rep.cover("rtabmap_offline", "live_ingest", "rtabmap_live", *(["nav2"] if c.get("check_nav2", True) else []))
    if not c.get("enabled", True):
        rep.add(G, "ROS 2", SKIP, "disabled in hub_config.json on this machine", gates=base)
        return
    if IS_WINDOWS:
        rc, out, err = run(["wsl.exe"] + (["-d", c["wsl_distro"]] if c["wsl_distro"] else []) + ["-e", "true"], 30)
        if rc != 0:
            rep.add(G, "WSL", FAIL, f"cannot start WSL (rc {rc}): {err[:200]}",
                    fix="install WSL2 + Ubuntu 24.04 (wsl --install -d Ubuntu-24.04), or set ros2.wsl_distro", gates=base)
            return
    pkgs = ROS_PKGS_REQUIRED + ROS_PKGS_OPTIONAL + (ROS_PKGS_NAV2 if c.get("check_nav2", True) else [])
    script = (ROS_SCRIPT.replace("@SETUP@", c["setup"]).replace("@WS@", c["workspace"])
              .replace("@PKGS@", " ".join(pkgs)).replace("@PORT@", str(cfg["phone_port"])).replace("@MAPS@", c["maps_dir"]))
    rc, out, err = bash_in_ros_env(script, cfg)
    f = parse_facts(out)
    if not f:
        rep.add(G, "ROS 2 probe", FAIL, f"no output from the ROS environment (rc {rc}): {err[-300:]}", gates=base)
        return
    rep.add(G, "Linux distro", OK if "24.04" in f.get("distro", "") else WARN, f.get("distro") or "unknown",
            fix="the setup in this project was Ubuntu 24.04 + Jazzy")
    if f.get("setup") != "1":
        rep.add(G, "ROS 2 install", FAIL, f"{c['setup']} not found",
                fix="sudo apt install ros-jazzy-desktop ros-dev-tools (see PROJECT_SUMMARY.md, section ROS 2 on WSL2)", gates=base)
        return
    rep.add(G, "ROS 2 install", OK if f.get("ros_distro") == cfg["ros2"].get("distro", "jazzy") or f.get("ros_distro") == "jazzy" else WARN,
            f"ROS_DISTRO={f.get('ros_distro')}, ros2 at {f.get('ros2') or 'NOT ON PATH'}")
    if not f.get("ros2"):
        rep.add(G, "ros2 command", FAIL, "ros2 not on PATH after sourcing setup.bash", gates=base)
        return

    rtab = ["rtabmap_launch", "rtabmap_slam", "rtabmap_odom", "rtabmap_viz"]
    miss = [p for p in rtab if not f.get(f"pkg:{p}")]
    rep.add(G, "RTAB-Map packages", FAIL if miss else OK, ("missing: " + ", ".join(miss)) if miss else "rtabmap_ros present",
            fix="sudo apt install ros-jazzy-rtabmap-ros" if miss else "", gates=["rtabmap_offline", "rtabmap_live"])
    ws_ok = f.get("ws") == "1"
    rep.add(G, "navsense workspace", OK if ws_ok else FAIL, c["workspace"] + ("" if ws_ok else " (install/setup.bash missing)"),
            fix="" if ws_ok else "cd ~/navsense_ws && colcon build --packages-select navsense_bridge --symlink-install",
            gates=["rtabmap_offline", "live_ingest"])
    exes = f.get("exe", "")
    for exe, stg in (("bridge", "live_ingest"), ("replay", "rtabmap_offline")):
        has = f.get("pkg:navsense_bridge") and re.search(rf"\b{exe}\b", exes)
        rep.add(G, f"navsense_bridge '{exe}'", OK if has else FAIL, "installed" if has else "not found",
                fix="" if has else "extract the latest navsense_bridge.tar.gz into ~/navsense_ws/src and colcon build", gates=[stg])
    deps = dict(kv.split("=", 1) for kv in f.get("pydeps", "").split(";") if "=" in kv)
    for mod, stg, why in (("av", "live_ingest", "the bridge decodes the phone's H.264 with PyAV"),
                          ("cv2", "rtabmap_offline", "the replay node uses OpenCV"),
                          ("numpy", "live_ingest", ""), ("yaml", "live_ingest", "")):
        have = deps.get(mod, "-") != "-"
        rep.add(G, f"python3 {mod} (WSL)", OK if have else FAIL, deps.get(mod, "-") if have else "missing " + why,
                fix="" if have else ("pip install --user --break-system-packages av" if mod == "av" else f"sudo apt install python3-{'opencv' if mod == 'cv2' else mod}"),
                gates=[stg] if not have else [])
    d = f.get("daemon", "")
    rep.add(G, "ROS 2 daemon", OK if "running" in d.lower() and "not" not in d.lower() else FAIL, d or "no answer",
            fix="ros2 daemon stop; ros2 daemon start   (this exact problem cost us hours once)", gates=base)
    rep.add(G, "ros2 topic list", OK if f.get("topic_rc") == "0" else FAIL,
            "works" if f.get("topic_rc") == "0" else f"failed/timed out (rc {f.get('topic_rc')})",
            fix="restart the daemon: ros2 daemon stop; ros2 daemon start", gates=base)
    rep.add(G, "GUI (WSLg)", OK if f.get("display") else WARN, f.get("display") or "no DISPLAY/WAYLAND_DISPLAY: rtabmap_viz / rviz2 won't open",
            fix="Windows 11 WSLg should provide it; try wsl --update")
    rep.add(G, "maps folder", OK if f.get("maps") == "1" else WARN, c["maps_dir"], fix="mkdir -p ~/maps")
    for p in ROS_PKGS_OPTIONAL:
        rep.add(G, f"optional pkg {p}", OK if f.get(f"pkg:{p}") else WARN, "present" if f.get(f"pkg:{p}") else "missing",
                fix=f"sudo apt install ros-jazzy-{p.replace('_', '-')}")
    if c.get("check_nav2", True):
        missn = [p for p in ROS_PKGS_NAV2 if not f.get(f"pkg:{p}")]
        rep.add(G, "Nav2 packages", FAIL if missn else OK, ("missing: " + ", ".join(missn)) if missn else "present",
                fix="sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup" if missn else "", gates=["nav2"])
    port = f.get("port", "")
    rep.add(G, f"TCP port {cfg['phone_port']}", OK if port == "free" else FAIL, "free" if port == "free" else port[-120:],
            fix="" if port == "free" else "something already listens on it (the Windows receiver? a previous bridge?)", gates=["live_ingest"])
    if IS_WINDOWS:
        check_windows_wsl_network(cfg, rep, G)


def check_windows_wsl_network(cfg, rep, G):
    port = cfg["phone_port"]
    prof = os.environ.get("USERPROFILE", "")
    txt = ""
    try:
        txt = open(os.path.join(prof, ".wslconfig"), encoding="utf-8", errors="replace").read()
    except OSError:
        pass
    mirrored = bool(re.search(r"networkingMode\s*=\s*mirrored", txt, re.I))
    rep.add(G, "WSL mirrored networking", OK if mirrored else WARN,
            "networkingMode=mirrored" if mirrored else "not set in %UserProfile%\\.wslconfig (the phone cannot reach WSL without it)",
            fix="" if mirrored else "[wsl2] networkingMode=mirrored  then: wsl --shutdown", gates=[])
    ps = lambda s: run(["powershell", "-NoProfile", "-Command", s], 25)  # noqa: E731
    rc, out, _ = ps(f"(Get-NetFirewallRule -DisplayName 'NavSense {port}' -ErrorAction SilentlyContinue | Measure-Object).Count")
    rep.add(G, "Windows firewall rule", OK if out.strip() not in ("", "0") and rc == 0 else WARN,
            f"rule 'NavSense {port}' present" if out.strip() not in ("", "0") and rc == 0 else f"rule 'NavSense {port}' not found",
            fix=f"admin PowerShell: New-NetFirewallRule -DisplayName 'NavSense {port}' -Direction Inbound -Protocol TCP -LocalPort {port} -Action Allow")
    rc, out, _ = ps(f"(Get-NetFirewallHyperVRule -DisplayName 'NavSense {port}' -ErrorAction SilentlyContinue | Measure-Object).Count")
    good = rc == 0 and out.strip() not in ("", "0")
    rep.add(G, "Hyper-V firewall rule", OK if good else WARN,
            f"rule 'NavSense {port}' present" if good else "not found, or not queryable without admin rights",
            fix=f"admin PowerShell: New-NetFirewallHyperVRule -DisplayName 'NavSense {port}' -Direction Inbound "
                f"-VMCreatorId '{{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}}' -Protocol TCP -LocalPorts {port}")


# ----------------------------------------------------------------------------------------------- placeholders
def check_external_tool(key, label, cfg, rep, smoke, stage):
    c = cfg[key]
    rep.cover(stage)
    if not c.get("enabled"):
        rep.add(label, label, TODO,
                f"PLACEHOLDER: {label} is not configured on this machine. Fill the '{key}' block in hub_config.json "
                f"(and extend check_{key}() in doctor.py if you need more than path/command checks).",
                gates=[stage])
        return
    paths = c.get("required_paths", [])
    if not paths:
        rep.add(label, "required paths", WARN, "enabled, but required_paths is empty: nothing is being verified")
    for p in paths:
        ok = os.path.exists(to_local_path(os.path.expandvars(os.path.expanduser(p))))
        rep.add(label, f"path {p}", OK if ok else FAIL, "exists" if ok else "missing", gates=[] if ok else [stage])
    sc = c.get("smoke_command") or []
    if sc and smoke:
        rc, out, err = run(sc, 300)
        good = rc in c.get("smoke_expect_rc", [0])
        rep.add(label, "smoke command", OK if good else FAIL, f"rc {rc}: {(err or out)[-200:]}", gates=[] if good else [stage])
    elif sc:
        rep.add(label, "smoke command", SKIP, "configured; run with --smoke")
    if not rep_has_gate(rep, label, stage):
        rep.add(label, label, OK, "configured")


def rep_has_gate(rep, group, stage):
    return any(ch.group == group and stage in ch.gates for ch in rep.checks)


def check_orbslam3(cfg, rep, smoke):
    """Check the local NavSense RGB-D SLAM installation without running a session."""
    group, gate = "ORB-SLAM3", ["orbslam3"]
    rep.cover("orbslam3")

    repo = Path(__file__).resolve().parent.parent
    base = repo / "experiments" / "orbslam3_rgbd_baseline"
    binary = base / "build" / "test_sample1_rgbd"
    runner = repo / "NavSenseHub" / "run_orbslam3.py"
    settings = base / "orbslam3_tecno.yaml"
    vocabulary = Path.home() / "opt/orbslam-deps/ORB_SLAM3/Vocabulary/ORBvoc.txt"

    for name, path, executable in (
        ("session runner", runner, False),
        ("RGB-D executable", binary, True),
        ("Tecno settings", settings, False),
        ("ORB vocabulary", vocabulary, False),
    ):
        good = path.is_file() and (not executable or os.access(path, os.X_OK))
        rep.add(group, name, OK if good else FAIL, str(path),
                fix="Build ORB-SLAM3 or correct the expected path" if not good else "",
                gates=[] if good else gate)

    # The runner passes these compatibility directories via LD_LIBRARY_PATH.
    lib_root = Path.home() / "opt/orbslam-deps"
    lib_paths = [
        lib_root / "openjph-0.31-compat/usr/lib",
        lib_root / "openexr-3.4-compat/usr/lib",
        lib_root / "opencv-4.6.0-install/lib",
    ]
    if binary.is_file() and os.access(binary, os.X_OK):
        env = os.environ.copy()
        env["LD_LIBRARY_PATH"] = os.pathsep.join(
            [str(p) for p in lib_paths] + ([env["LD_LIBRARY_PATH"]] if env.get("LD_LIBRARY_PATH") else [])
        )
        try:
            result = subprocess.run(["ldd", str(binary)], capture_output=True, text=True,
                                    timeout=15, env=env)
            missing = [line.strip() for line in (result.stdout + "\n" + result.stderr).splitlines()
                       if "not found" in line]
            good = result.returncode == 0 and not missing
            detail = "; ".join(missing) if missing else ("all shared libraries resolved" if good else result.stderr.strip()[-300:])
            rep.add(group, "shared libraries", OK if good else FAIL, detail,
                    fix="Check the OpenEXR/OpenCV compatibility library paths" if not good else "",
                    gates=[] if good else gate)
        except (OSError, subprocess.TimeoutExpired) as exc:
            rep.add(group, "shared libraries", FAIL, str(exc), gates=gate)
    else:
        rep.add(group, "shared libraries", FAIL, "executable unavailable", gates=gate)

    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        rep.add(group, "viewer display", WARN, "no DISPLAY/WAYLAND_DISPLAY; Pangolin viewer may not open")


def check_vlm(cfg, rep, smoke):
    c = cfg["vlm"]
    G, gate = "VLM", ["vlm"]
    rep.cover("vlm")
    
    if not c.get("enabled"):
        rep.add(G, "VLM", SKIP, "disabled in hub_config.json on this machine", gates=gate)
        return

    # 1. Check the python interpreter
    cmd = [c["python"]] if c.get("python") else [sys.executable]
    py_name = c.get("python") or "current interpreter"
    
    # 2. Probe for the exact modules your script needs
    probe_code = "import torch; import open_clip; import cv2; import PIL; import numpy; print('ok')"
    rc, out, err = run(cmd + ["-c", probe_code], 120)   # 120s: import torch on Windows can be slow
    if rc == 0:
        rep.add(G, "Python dependencies", OK, f"open_clip, torch, cv2 found in {py_name}")
    else:
        # show the actual reason so it's diagnosable from the hub log alone
        reason = (err or out or "no output").strip().splitlines()
        reason = reason[-1] if reason else f"probe exited with rc {rc}"
        rep.add(G, "Python dependencies", FAIL,
                f"probe failed in {py_name} (rc {rc}): {reason}",
                fix="pip install open_clip_torch torch opencv-python (and check the error above)",
                gates=gate)

    # 3. Check for the script
    sp = c.get("script_path") or os.path.join(os.path.dirname(__file__), "vlm_openclip.py")
    if os.path.isfile(sp):
        rep.add(G, "VLM script", OK, os.path.basename(sp))
    else:
        rep.add(G, "VLM script", FAIL, f"not found at {sp}", fix="Create vlm_openclip.py or set vlm.script_path", gates=gate)

    # 4. Smoke test if requested
    sc = c.get("smoke_command") or []
    if sc and smoke:
        rc, out, err = run(sc, 300)
        good = rc in c.get("smoke_expect_rc", [0])
        rep.add(G, "smoke command", OK if good else FAIL, f"rc {rc}: {(err or out)[-200:]}", gates=[] if good else [gate])
    elif sc:
        rep.add(G, "smoke command", SKIP, "configured; run with --smoke")

    if not rep_has_gate(rep, G, "vlm"):
        rep.add(G, "VLM", OK, "ready")


def check_local_python(rep):
    """The 'prepare' stage (EuRoC export etc.) only needs numpy + OpenCV in the Python running the receiver/hub."""
    rep.cover("prepare")
    for mod in ("numpy", "cv2", "av"):
        try:
            m = __import__(mod)
            rep.add("This machine", f"python {mod}", OK, str(getattr(m, "__version__", "ok")))
        except Exception:
            rep.add("This machine", f"python {mod}", WARN, "not installed here",
                    fix=f"pip install {'opencv-python' if mod == 'cv2' else mod}")


# ----------------------------------------------------------------------------------------------- output
COLORS = {OK: "32", WARN: "33", FAIL: "31", TODO: "35", SKIP: "90"}
TAGS = {OK: " OK ", WARN: "WARN", FAIL: "FAIL", TODO: "TODO", SKIP: "SKIP"}


def print_report(rep, cfg_note, color, verbose):
    def c(s, st):
        return f"\033[{COLORS[st]}m{s}\033[0m" if color else s
    print(f"NavSense doctor | {platform.node()} | {platform.system()}{' (WSL)' if IS_WSL else ''} | {cfg_note}")
    group = None
    for ch in rep.checks:
        if ch.group != group:
            group = ch.group
            print(f"\n== {group}")
        print(f"  [{c(TAGS[ch.status], ch.status)}] {ch.name}" + (f": {ch.detail}" if ch.detail else ""))
        if ch.fix and ch.status in (FAIL, WARN, TODO):
            print(f"         fix: {ch.fix}")
    print("\n== Stage readiness on this machine")
    for st, v in rep.stage_state().items():
        tag = (c("NOT CHECKED", SKIP) if v["ready"] is None else
               c("READY      ", OK) if v["ready"] else c("NOT READY  ", FAIL))
        print(f"  {tag} {st}")
        if v["ready"] is False and verbose:
            for r in v["blocked_by"]:
                print(f"            - {r}")
    if not verbose:
        print("  (use --verbose to see what blocks each stage)")


def main():
    ap = argparse.ArgumentParser(description="NavSense doctor")
    ap.add_argument("--config", default=None)
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--out", default=None, help="also write the JSON report to this file")
    ap.add_argument("--only", default="", help="comma list: da3,ros2,orbslam3,vlm")
    ap.add_argument("--smoke", action="store_true")
    ap.add_argument("--verbose", "-v", action="store_true")
    ap.add_argument("--no-color", action="store_true")
    ap.add_argument("--init-config", action="store_true")
    a = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    cfg_path = a.config or next((p for p in (os.path.join(os.getcwd(), "hub_config.json"),
                                             os.path.join(here, "hub_config.json")) if os.path.isfile(p)), None)
    if a.init_config:
        target = a.config or os.path.join(here, "hub_config.json")
        if os.path.exists(target):
            print(f"{target} already exists; not overwriting")
            return 0
        with open(target, "w", encoding="utf-8") as f:
            json.dump(DEFAULT_CONFIG, f, indent=2)
        print(f"wrote {target}: edit the paths, then run doctor.py again")
        return 0
    cfg, note = load_config(cfg_path)
    only = {x.strip().lower() for x in a.only.split(",") if x.strip()}
    want = lambda k: not only or k in only  # noqa: E731

    rep = Report()
    check_local_python(rep)
    if want("da3"):
        check_da3(cfg, rep, a.smoke)
    if want("ros2"):
        check_ros2(cfg, rep)
    if want("orbslam3"):
        check_orbslam3(cfg, rep, a.smoke)
    if want("vlm"):
        check_vlm(cfg, rep, a.smoke)

    result = {
        "doctor_version": "0.1",
        "time_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "machine": {"hostname": platform.node(), "os": platform.platform(), "wsl": IS_WSL,
                    "python": sys.version.split()[0]},
        "config": note,
        "checks": [asdict(c) for c in rep.checks],
        "stages": rep.stage_state(),
    }
    if a.out:
        with open(a.out, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2)
    if a.json:
        print(json.dumps(result, indent=2))
    else:
        if IS_WINDOWS:
            os.system("")   # enables ANSI colours in recent Windows terminals
        print_report(rep, note, sys.stdout.isatty() and not a.no_color, a.verbose)
        if a.out:
            print(f"\nreport written to {a.out}")
    return 1 if any(c.status == FAIL for c in rep.checks) else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
    except Exception as e:  # noqa: BLE001
        print(f"doctor crashed: {e!r}", file=sys.stderr)
        sys.exit(2)