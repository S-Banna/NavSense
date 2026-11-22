#!/usr/bin/env python3
"""
NavSense manifest generator (contract v1.0)

  python generate_manifest.py --session <session_folder> [--params params.json] [--dry-run]

Builds (or refreshes) <session>/manifest.json from the files the receiver already wrote:
  imu.jsonl, encoder.csv, events.jsonl, clock_sync.json, session_meta.json, session_info.json,
  video.h264/video.jsonl, mav0/**, timestamps.txt, sync_report.txt, depth/, calibration/.

Rules it follows (SESSION_CONTRACT section 4):
  * Raw data is never touched. Only manifest.json is written, atomically (tmp file + os.replace).
  * It re-reads the existing manifest first and keeps everything it does not own:
    other stages' entries (depth_da3, rtabmap_offline, ...), calibration entries, bundles, source
    edits, label, created_utc. It owns: time_base, streams, and the `prepare` stage entry.
  * Idempotent: running it twice gives the same result.
  * Tolerates partial sessions: a missing stream is reported as present=false.
  * Validates against manifest.schema.json (needs `pip install jsonschema`; without it only a
    minimal required-keys check runs). A manifest that fails validation is NOT written unless --force.

Exit codes: 0 ok, 1 error (bad session folder, unreadable files), 3 validation failed.
Standard library only (jsonschema optional).
"""
from __future__ import annotations

import argparse
import copy
import json
import os
import platform
import re
import socket
import statistics
import struct
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

CONTRACT_VERSION = "1.0"
GENERATOR_VERSION = "0.1"
IMAGE_EXTS = {".jpg", ".jpeg", ".png"}
PREPARE_REQUIRED = ["mav0/cam0/data.csv", "mav0/imu0/data.csv", "timestamps.txt"]
PREPARE_OPTIONAL = ["orbslam3_phone.yaml", "sync_report.txt"]

DEFAULTS = {
    "recorder_version": "0.4",
    "left_pin": "D12",     # SESSION_CONTRACT / manifest.example.json. NOTE: PROJECT_SUMMARY says D3. Verify against the sketch.
    "right_pin": "D2",
    "robot_description": "L298N differential car, two single-channel wheel encoders",
    "arduino_sketch": "car_encoder.ino",
}

SEEDED_STAGES = {
    "depth_da3": {"status": "not_run"},
    "rtabmap_offline": {"status": "not_run"},
    "orbslam3": {
        "status": "placeholder",
        "owner": "TODO teammate",
        "todo": [
            "document the machine (OS, ORB-SLAM3 version/commit, build notes)",
            "state which mode is run (mono / mono-inertial / RGB-D) and the exact command line",
            "map the settings yaml (intrinsics, IMU noise, Tbc) to the contract's calibration/ files",
            "write the outputs under slam/orbslam3/ (trajectory_tum.txt + trajectory_meta.json at minimum)",
            "fill the 'orbslam3' block of hub_config.json so doctor.py can verify the install",
        ],
        "inputs": ["mav0/cam0/data", "mav0/cam0/data.csv", "mav0/imu0/data.csv", "timestamps.txt",
                   "orbslam3_phone.yaml"],
    },
    "eval": {"status": "not_run"},
    "live_ingest": {"status": "not_run"},
    "rtabmap_live": {"status": "not_run"},
    "nav2": {"status": "not_run"},
    "vlm": {
        "status": "placeholder",
        "owner": "TODO",
        "todo": ["choose the model and how it runs (local vs API)",
                 "define input (images/map + text query) and output (goal pose in map frame + confidence)",
                 "decide where results live (proposal: semantic/landmarks.json)"],
    },
}

WARNINGS: list[str] = []


def warn(msg: str):
    WARNINGS.append(msg)
    print(f"WARNING: {msg}")


# ------------------------------------------------------------------------------------------ small helpers
def iso_utc(ts: float) -> str:
    return datetime.fromtimestamp(ts, timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def read_json(path: Path, default=None):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return default
    except Exception as e:  # noqa: BLE001
        warn(f"could not parse {path.name}: {e}")
        return default


def atomic_write_json(path: Path, data):
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    for attempt in range(10):          # Windows: os.replace can briefly fail if another process has the file open
        try:
            os.replace(tmp, path)
            return
        except PermissionError:
            if attempt == 9:
                raise
            time.sleep(0.2)


def image_size(path: Path):
    """(width, height) of a JPEG/PNG read from the header, no imaging libraries needed."""
    try:
        with open(path, "rb") as f:
            head = f.read(26)
            if head[:8] == b"\x89PNG\r\n\x1a\n":
                w, h = struct.unpack(">II", head[16:24])
                return w, h
            if head[:2] == b"\xff\xd8":
                f.seek(2)
                while True:
                    b = f.read(1)
                    if not b:
                        return None
                    if b != b"\xff":
                        continue
                    m = f.read(1)
                    while m == b"\xff":
                        m = f.read(1)
                    if not m:
                        return None
                    mk = m[0]
                    if mk in (0xD8, 0x01) or 0xD0 <= mk <= 0xD7:
                        continue
                    seg = f.read(2)
                    if len(seg) < 2:
                        return None
                    ln = struct.unpack(">H", seg)[0]
                    if mk in (0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF):
                        d = f.read(5)
                        _, h, w = struct.unpack(">BHH", d)
                        return w, h
                    f.seek(ln - 2, 1)
    except Exception:  # noqa: BLE001
        pass
    return None


def rate_hz(first, last, n):
    if n is None or n < 2 or first is None or last is None or last <= first:
        return None
    return round((n - 1) / ((last - first) / 1e9), 2)


# ------------------------------------------------------------------------------------------ scanners
def scan_frames(session: Path, session_meta: dict, session_info: dict):
    cam_rel = "mav0/cam0/data"
    cam_dir = session / cam_rel
    rgb = {"present": False, "path": cam_rel}
    stamps: list[int] = []
    exts: dict[str, int] = {}
    first_file = None
    if cam_dir.is_dir():
        with os.scandir(cam_dir) as it:
            for e in it:
                p = Path(e.name)
                if p.suffix.lower() in IMAGE_EXTS and p.stem.isdigit():
                    stamps.append(int(p.stem))
                    ext = "jpg" if p.suffix.lower() in (".jpg", ".jpeg") else "png"
                    exts[ext] = exts.get(ext, 0) + 1
                    first_file = first_file or Path(e.path)
    stamps.sort()
    if not stamps:
        return rgb, []
    rgb["present"] = True
    rgb["format"] = max(exts, key=exts.get)
    if len(exts) > 1:
        warn(f"mixed frame formats in {cam_rel}: {exts}")
    rgb["count"] = len(stamps)
    size = image_size(first_file)
    if size:
        rgb["width"], rgb["height"] = size
    for k in ("sensor_orientation", "rotation_cw_deg"):
        if isinstance(session_meta.get(k), int):
            rgb[k] = session_meta[k]
    diffs = [b - a for a, b in zip(stamps, stamps[1:])]
    if diffs:
        med = statistics.median(diffs)
        if med > 0:
            rgb["fps_median"] = round(1e9 / med, 2)
        rgb["max_gap_ms"] = round(max(diffs) / 1e6, 1)
    if len(stamps) != len(set(stamps)):
        warn("duplicate frame timestamps found")
    rgb["first_ts_ns"], rgb["last_ts_ns"] = stamps[0], stamps[-1]
    if isinstance(session_info.get("phone_dropped_frames"), int):
        rgb["dropped_by_phone"] = session_info["phone_dropped_frames"]
    written = session_info.get("frames_written")
    if isinstance(written, int) and written != len(stamps):
        warn(f"session_info says {written} frames written but {len(stamps)} are on disk")
    if session_info.get("frames_lost_by_writer"):
        warn(f"receiver writer dropped {session_info['frames_lost_by_writer']} frames")
    return rgb, stamps


def scan_imu(session: Path, session_info: dict):
    path = session / "imu.jsonl"
    imu = {"present": False}
    info = {"first": None, "last": None}
    if not path.is_file():
        return imu, info
    acc_t: list[int] = []
    gyr_t: list[int] = []
    zero = [0, 0, 0]
    bad_lines = 0
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            try:
                r = json.loads(line)
                s, t = r["sensor"], int(r["t_ns"])
            except Exception:  # noqa: BLE001
                bad_lines += 1       # a truncated last line after a hard kill is normal
                continue
            if s == "accelerometer":
                acc_t.append(t)
            elif s == "gyroscope":
                gyr_t.append(t)
                v = r.get("v") or [None] * 3
                for i in range(3):
                    if v[i] == 0.0:
                        zero[i] += 1
    if bad_lines > 1:
        warn(f"{bad_lines} unreadable lines in imu.jsonl")
    if not acc_t and not gyr_t:
        return imu, info
    imu["present"] = True
    imu["raw_path"] = "imu.jsonl"
    if (session / "mav0/imu0/data.csv").is_file():
        imu["merged_path"] = "mav0/imu0/data.csv"
    a_hz = rate_hz(min(acc_t), max(acc_t), len(acc_t)) if acc_t else None
    g_hz = rate_hz(min(gyr_t), max(gyr_t), len(gyr_t)) if gyr_t else None
    if a_hz:
        imu["accel_hz"] = a_hz
    if g_hz:
        imu["gyro_hz"] = g_hz
    merged_hz = merged_rate(session / "mav0/imu0/data.csv")
    if merged_hz:
        imu["merged_hz"] = merged_hz
    # same heuristics as the receiver's live check
    verdict = "ok"
    reasons = []
    if not gyr_t:
        verdict = "missing"
        reasons.append("no gyro samples")
    else:
        ts = sorted(set(gyr_t))
        d = [b - a for a, b in zip(ts, ts[1:])]
        if len(gyr_t) > 150 and d:
            jitter_us = statistics.pstdev(d) / 1e3
            period = statistics.median(d)
            off = period % 500_000                       # distance from a multiple of 0.5 ms
            round_period = min(off, 500_000 - off) < 20_000
            if jitter_us < 5 and round_period:
                verdict = "fake"
                reasons.append(f"gyro timestamps perfectly regular at a round {period/1e6:.3f} ms period")
            elif jitter_us < 5:
                reasons.append(f"timestamps very regular (jitter {jitter_us:.1f} us) but period {period/1e6:.4f} ms is not round and axes vary: likely sensor-hub clock, treated as real")
            dead = [("x", "y", "z")[i] for i in range(3) if zero[i] / len(gyr_t) > 0.9]
            if dead:
                verdict = "fake"
                reasons.append(f"gyro axes {dead} read exactly 0")
        elif len(gyr_t) <= 150:
            verdict = "unknown"
            reasons.append("too few gyro samples to judge")
    problems = " | ".join(session_info.get("imu_problems") or [])
    if verdict == "ok" and problems:
        if "NO GYRO" in problems.upper():
            verdict = "missing"
            reasons.append("receiver flagged: " + problems)
        else:
            reasons.append("receiver note (not used for the verdict): " + problems)
    imu["gyro_verdict"] = verdict
    imu["units"] = "gyro rad/s, accel m/s^2 (with gravity), Android device frame"
    if verdict != "ok":
        warn(f"gyro_verdict={verdict} ({'; '.join(reasons)}): not usable for visual-inertial work")
    allt = acc_t + gyr_t
    info["first"], info["last"] = min(allt), max(allt)
    return imu, info


def merged_rate(path: Path):
    """Rate of mav0/imu0/data.csv from its first/last timestamps and row count."""
    if not path.is_file():
        return None
    n, first, last = 0, None, None
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            try:
                t = int(line.split(",", 1)[0])
            except ValueError:
                continue
            first = t if first is None else first
            last = t
            n += 1
    return rate_hz(first, last, n)


def scan_encoders(session: Path, left_pin: str, right_pin: str):
    path = session / "encoder.csv"
    enc = {"present": False}
    info = {"first": None, "last": None}
    if not path.is_file():
        return enc, info
    n, t_first, t_last, a_first, a_last = 0, None, None, None, None
    with open(path, encoding="utf-8", errors="replace") as f:
        next(f, None)                                        # header: t_ns,arduino_ms,left,right,laptop_rx_ms
        for line in f:
            p = line.strip().split(",")
            if len(p) < 4:
                continue
            try:
                t, a = int(p[0]), float(p[1])
            except ValueError:
                continue
            t_first = t if t_first is None else t_first
            t_last = t
            a_first = a if a_first is None else a_first
            a_last = a
            n += 1
    if n == 0:
        return enc, info
    enc.update({"present": True, "path": "encoder.csv", "count": n,
                "left_pin": left_pin, "right_pin": right_pin, "edge": "rising", "directional": False})
    if n > 1 and a_last > a_first:
        enc["rate_hz"] = round((n - 1) / ((a_last - a_first) / 1000.0), 2)   # arduino millis: exact spacing
    info["first"], info["last"] = t_first, t_last
    return enc, info


def scan_events(session: Path):
    """Phone info from a `hello` event if the receiver stored it (see the receiver patch)."""
    out = {}
    path = session / "events.jsonl"
    if not path.is_file():
        return out
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            try:
                r = json.loads(line)
            except Exception:  # noqa: BLE001
                continue
            if r.get("type") == "hello":
                phone = {}
                for key, names in (("manufacturer", ("manufacturer", "make", "brand")),
                                   ("model", ("model", "device", "device_model")),
                                   ("android_sdk", ("android_sdk", "sdk", "sdk_int")),
                                   ("app_version", ("app_version", "app", "version"))):
                    for nme in names:
                        if r.get(nme) not in (None, ""):
                            phone[key] = r[nme]
                            break
                if "android_sdk" in phone:
                    try:
                        phone["android_sdk"] = int(phone["android_sdk"])
                    except (TypeError, ValueError):
                        del phone["android_sdk"]
                for k in ("manufacturer", "model", "app_version"):
                    if k in phone:
                        phone[k] = str(phone[k])
                out["phone"] = phone
                break
    return out


def scan_depth(session: Path):
    d = session / "depth"
    depth = {"present": False}
    if not d.is_dir():
        return depth
    n = sum(1 for p in d.iterdir() if p.suffix.lower() == ".png" and p.stem.isdigit())
    if n:
        depth = {"present": True, "path": "depth", "count": n}
        if (d / "depth_meta.json").is_file():
            depth["meta"] = "depth/depth_meta.json"
    return depth


# ------------------------------------------------------------------------------------------ prepare stage
def build_prepare(session: Path, old: dict | None, recorder_version: str):
    present = [r for r in PREPARE_REQUIRED + PREPARE_OPTIONAL if (session / r).is_file()]
    missing = [r for r in PREPARE_REQUIRED if r not in present]
    stage = {
        "machine": {"hostname": socket.gethostname(), "os": platform.platform()},
        "tool": {"name": "navsense_receiver export_euroc", "version": recorder_version},
        "inputs": [p for p in ("imu.jsonl", "mav0/cam0/data") if (session / p).exists()],
        "outputs": present,
    }
    if present:
        stage["finished_utc"] = iso_utc(max((session / p).stat().st_mtime for p in present))
    if (session / "sync_report.txt").is_file():
        stage["log"] = "sync_report.txt"
        text = (session / "sync_report.txt").read_text(encoding="utf-8", errors="replace")
        metrics = {}
        m = re.search(r"largest gap between frames: (\d+) ms", text)
        if m:
            metrics["largest_frame_gap_ms"] = int(m.group(1))
        m = re.search(r"IMU samples between consecutive frames: min (\d+)", text)
        if m:
            metrics["imu_samples_between_frames_min"] = int(m.group(1))
        if metrics:
            stage["metrics"] = metrics
    if not missing:
        stage["status"] = "done"
    elif not present:
        stage["status"] = "not_run"
        stage["notes"] = "no EuRoC export files yet (receiver ran with --no-export, or the session has no frames/IMU)"
    else:
        stage["status"] = "failed"
        stage["error"] = "missing export files: " + ", ".join(missing)
    if old:                                                  # keep what a human or another tool added
        for k in ("params", "notes"):
            if k in old and k not in stage:
                stage[k] = old[k]
    return {"status": stage.pop("status"), **stage}


# ------------------------------------------------------------------------------------------ manifest
def build_manifest(session: Path, existing: dict | None, opts: dict):
    session_meta = read_json(session / "session_meta.json", {}) or {}
    session_info = read_json(session / "session_info.json", {}) or {}
    m = copy.deepcopy(existing) if existing else {}

    sid = re.sub(r"[^A-Za-z0-9_.-]", "_", session.name)
    m["contract_version"] = CONTRACT_VERSION
    m["session_id"] = sid
    label = m.get("label") or opts.get("label")
    if label:
        m["label"] = label
    if not m.get("created_utc"):
        try:
            m["created_utc"] = (datetime.strptime(sid, "%Y%m%d_%H%M%S").astimezone(timezone.utc)
                                .isoformat(timespec="seconds").replace("+00:00", "Z"))
        except ValueError:
            m["created_utc"] = iso_utc(session.stat().st_mtime)

    # source: defaults < what we can read from events < what is already in the manifest
    derived = {
        "kind": "robot_phone_recording",
        "recorder": {"name": "navsense_receiver", "version": opts["recorder_version"]},
        "robot": {"description": opts["robot_description"], "arduino_sketch": opts["arduino_sketch"]},
    }
    ev = scan_events(session)
    if ev.get("phone"):
        derived["phone"] = ev["phone"]
    src = derived
    for k, v in (m.get("source") or {}).items():
        if isinstance(v, dict) and isinstance(src.get(k), dict):
            src[k].update(v)
        else:
            src[k] = v
    m["source"] = src

    rgb, stamps = scan_frames(session, session_meta, session_info)
    imu, imu_t = scan_imu(session, session_info)
    enc, enc_t = scan_encoders(session, opts["left_pin"], opts["right_pin"])
    video = {"present": (session / "video.h264").is_file()}
    if video["present"]:
        video.update({"path": "video.h264", "codec": "h264"})
        if (session / "video.jsonl").is_file():
            video["index"] = "video.jsonl"

    streams = m.setdefault("streams", {})
    streams.update({"rgb": rgb, "video_raw": video, "imu": imu, "encoders": enc})
    depth_stage = ((m.get("stages") or {}).get("depth_da3") or {}).get("status")
    if depth_stage != "running":                              # don't fight a DA3 run in progress
        streams["depth"] = scan_depth(session)

    # time_base
    firsts = [x for x in (stamps[0] if stamps else None, imu_t["first"], enc_t["first"]) if x is not None]
    lasts = [x for x in (stamps[-1] if stamps else None, imu_t["last"], enc_t["last"]) if x is not None]
    tb = {"clock": "phone_elapsedRealtime", "unit": "ns"}
    if firsts:
        tb["t0_ns"], tb["t_end_ns"] = min(firsts), max(lasts)
    if (session / "clock_sync.json").is_file():
        tb["clock_sync_file"] = "clock_sync.json"
    else:
        warn("clock_sync.json missing: timestamps cannot be related to laptop time")
    m["time_base"] = tb

    # calibration: seed only what is absent; never overwrite a calibration tool's entry
    cal = m.setdefault("calibration", {})
    cal.setdefault("camera", {
        "status": "placeholder", "path": "calibration/camera_info.yaml",
        "source": "intrinsics guessed from image size in orbslam3_phone.yaml (REPLACE with a checkerboard calibration)",
        "notes": "until calibrated, DA3 falls back to orbslam3_phone.yaml"})
    cal.setdefault("imu_camera", {
        "status": "placeholder", "path": "calibration/extrinsics.json",
        "source": "rotation derived from Android axes; translation unknown; time offset 0"})
    cal.setdefault("wheel", {"status": "missing", "notes": "TODO ticks per metre (left/right) and effective track width"})

    # stages: own `prepare`, seed missing ones, normalise nulls written by other tools
    stages = m.setdefault("stages", {})
    stages["prepare"] = build_prepare(session, stages.get("prepare"), opts["recorder_version"])
    for name, seed in SEEDED_STAGES.items():
        stages.setdefault(name, copy.deepcopy(seed))
    for rec in stages.values():
        if isinstance(rec, dict):
            for k in [k for k, v in rec.items() if v is None]:   # schema wants strings; None is invalid
                del rec[k]

    if not rgb["present"]:
        warn("no RGB frames found: DA3/RTAB-Map cannot run on this session yet")
    if rgb.get("max_gap_ms", 0) >= 900:
        warn(f"largest frame gap {rgb['max_gap_ms']:.0f} ms: ORB-SLAM3 will likely reset its map")
    ordered = {k: m[k] for k in ("contract_version", "session_id", "label", "created_utc", "source", "time_base",
                                 "streams", "calibration", "stages", "bundles") if k in m}
    for k, v in m.items():
        ordered.setdefault(k, v)
    return ordered


# ------------------------------------------------------------------------------------------ validation
def find_schema(explicit: str | None):
    cands = [Path(explicit)] if explicit else []
    here = Path(__file__).resolve().parent
    cands += [here / "manifest.schema.json"] + [p / "manifest.schema.json" for p in here.parents[:3]]
    return next((c for c in cands if c.is_file()), None)


def validate(manifest: dict, schema_path: Path | None):
    if schema_path is None:
        errs = [f"missing required key: {k}" for k in
                ("contract_version", "session_id", "created_utc", "time_base", "streams", "stages") if k not in manifest]
        return errs, "minimal check (manifest.schema.json not found)"
    try:
        import jsonschema
    except ImportError:
        errs = [f"missing required key: {k}" for k in
                ("contract_version", "session_id", "created_utc", "time_base", "streams", "stages") if k not in manifest]
        return errs, "minimal check (pip install jsonschema for full validation)"
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    cls = jsonschema.validators.validator_for(schema)
    errs = [f"{'/'.join(str(x) for x in e.absolute_path) or '<root>'}: {e.message}"
            for e in cls(schema).iter_errors(manifest)]
    return errs, f"jsonschema against {schema_path}"


# ------------------------------------------------------------------------------------------ main
def summarize(m: dict):
    s = m["streams"]
    bits = []
    r = s.get("rgb", {})
    bits.append(f"rgb {r.get('count', 0)} frames" + (f" {r.get('width')}x{r.get('height')} @ {r.get('fps_median')} fps" if r.get("present") else ""))
    i = s.get("imu", {})
    bits.append("imu " + (f"{i.get('gyro_hz')} Hz gyro, verdict {i.get('gyro_verdict')}" if i.get("present") else "absent"))
    e = s.get("encoders", {})
    bits.append("encoders " + (f"{e.get('count')} samples" if e.get("present") else "absent"))
    d = s.get("depth", {})
    bits.append("depth " + (f"{d.get('count')} pngs" if d.get("present") else "absent"))
    bits.append(f"prepare={m['stages']['prepare']['status']}")
    return " | ".join(bits)


def generate(session: Path, opts: dict, dry_run=False, force=False, schema=None) -> int:
    existing_path = session / "manifest.json"
    existing = None
    if existing_path.is_file():
        try:
            existing = json.loads(existing_path.read_text(encoding="utf-8"))
        except Exception as e:  # noqa: BLE001
            backup = session / f"manifest.broken.{int(time.time())}.json"
            existing_path.replace(backup)
            warn(f"existing manifest.json was unreadable ({e}); moved to {backup.name} and regenerating")
    manifest = build_manifest(session, existing, opts)
    errors, how = validate(manifest, find_schema(schema))
    print(f"Session : {session}")
    print(f"Existing: {'yes (merged)' if existing else 'no (new)'}")
    print(f"Summary : {summarize(manifest)}")
    print(f"Validate: {how}: {'OK' if not errors else str(len(errors)) + ' problem(s)'}")
    for e in errors:
        print(f"   - {e}")
    if errors and not force:
        print("Not writing manifest.json (fix the problems above, or use --force).")
        return 3
    if dry_run:
        print("\nDry run: nothing written. Would write:", existing_path)
        return 0
    atomic_write_json(existing_path, manifest)
    print(f"\nWrote {existing_path}" + (f"  ({len(WARNINGS)} warning(s))" if WARNINGS else ""))
    return 0


def main():
    ap = argparse.ArgumentParser(description="Generate manifest.json for a NavSense session (contract v1.0)")
    ap.add_argument("--session", required=True, type=Path)
    ap.add_argument("--params", type=Path, help="JSON file: label, recorder_version, left_pin, right_pin, robot_description")
    ap.add_argument("--dry-run", action="store_true", help="check inputs and print the result, write nothing")
    ap.add_argument("--schema", help="path to manifest.schema.json (default: searched next to this script and its parents)")
    ap.add_argument("--label")
    ap.add_argument("--recorder-version")
    ap.add_argument("--force", action="store_true", help="write even if schema validation fails")
    a = ap.parse_args()

    session = a.session.resolve()
    if not session.is_dir():
        print(f"Session folder does not exist: {session}", file=sys.stderr)
        return 1
    opts = dict(DEFAULTS)
    if a.params:
        opts.update({k: v for k, v in (read_json(a.params, {}) or {}).items() if k in DEFAULTS or k == "label"})
    if a.label:
        opts["label"] = a.label
    if a.recorder_version:
        opts["recorder_version"] = a.recorder_version
    try:
        return generate(session, opts, a.dry_run, a.force, a.schema)
    except Exception as e:  # noqa: BLE001
        print(f"generate_manifest FAILED: {e!r}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
