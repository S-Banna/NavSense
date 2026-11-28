#!/usr/bin/env python3
"""
Install a camera calibration into NavSense sessions.

  py install_calibration.py sessions/20261119_115756                  # default camera profile
  py install_calibration.py sessions/20261119_115756 --camera tecno_cla6_720x1280_portrait
  py install_calibration.py --all sessions
  py install_calibration.py --all sessions --dry-run

Writes <session>/calibration/camera_info.yaml (ROS camera_info / ost.yaml layout, the format
da3_session_runner.py reads) and sets manifest.json calibration.camera to status "calibrated".
It checks the saved frame size matches the calibration size and refuses otherwise.
Add a profile to CAMERAS if you recalibrate. Default profile: infinix_x678b_1280x720_full.
Keeps a master copy next to this script: calibration/<profile-name>.yaml
"""
import argparse
import json
import os
import struct
import sys
from pathlib import Path

# Each profile: image size, intrinsics, distortion (OpenCV order k1,k2,p1,p2,k3)
CAMERAS = {
    "tecno_cla6_720x1280_portrait": {
        "width": 720, "height": 1280,
        "fx": 1139.39, "fy": 1127.84, "cx": 356.41, "cy": 625.81,
        "dist": [1.13168028e-01, 1.46371408e-01, 3.99940152e-03, -2.10289154e-03, -5.56445807e+00],
    },
    "infinix_x678b_1280x720_full": {
        "width": 1280, "height": 720,
        "fx": 886.06438921, "fy": 894.14965967, "cx": 637.2060913, "cy": 356.97463383,
        "dist": [0.2402168297, -0.891742, -0.0065811817, -0.0064338626, 0.0],
    },
}
DEFAULT_CAMERA = "infinix_x678b_1280x720_full"

def yaml_text(name, c):
    k = f"[{c['fx']}, 0.0, {c['cx']}, 0.0, {c['fy']}, {c['cy']}, 0.0, 0.0, 1.0]"
    p = f"[{c['fx']}, 0.0, {c['cx']}, 0.0, 0.0, {c['fy']}, {c['cy']}, 0.0, 0.0, 0.0, 1.0, 0.0]"
    d = "[" + ", ".join(repr(x) for x in c["dist"]) + "]"
    return "\n".join([
        f"image_width: {c['width']}", f"image_height: {c['height']}", f"camera_name: {name}",
        "camera_matrix:", "  rows: 3", "  cols: 3", f"  data: {k}",
        "distortion_model: plumb_bob",
        "distortion_coefficients:", "  rows: 1", "  cols: 5", f"  data: {d}",
        "rectification_matrix:", "  rows: 3", "  cols: 3", "  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]",
        "projection_matrix:", "  rows: 3", "  cols: 4", f"  data: {p}", ""])


def frame_size(session: Path):
    d = session / "mav0" / "cam0" / "data"
    if not d.is_dir():
        return None
    for p in d.iterdir():
        if p.suffix.lower() == ".png":
            with open(p, "rb") as f:
                return struct.unpack(">II", f.read(24)[16:24])
        if p.suffix.lower() in (".jpg", ".jpeg"):
            with open(p, "rb") as f:
                f.seek(2)
                while True:
                    if f.read(1) != b"\xff":
                        continue
                    m = f.read(1)
                    while m == b"\xff":
                        m = f.read(1)
                    if not m:
                        return None
                    if m[0] in (0xD8, 0x01) or 0xD0 <= m[0] <= 0xD7:
                        continue
                    ln = struct.unpack(">H", f.read(2))[0]
                    if m[0] in (0xC0, 0xC1, 0xC2, 0xC3):
                        _, h, w = struct.unpack(">BHH", f.read(5))
                        return w, h
                    f.seek(ln - 2, 1)
    return None


def install(session: Path, dry: bool, name: str, cal: dict):
    size = frame_size(session)
    if size is None:
        print(f"[skip] {session.name}: no frames found")
        return False
    if size != (cal["width"], cal["height"]):
        print(f"[skip] {session.name}: frames are {size[0]}x{size[1]}, calibration is {cal['width']}x{cal['height']}")
        return False
    if dry:
        print(f"[dry ] {session.name}: would write calibration/camera_info.yaml and update the manifest")
        return True
    cal_dir = session / "calibration"
    cal_dir.mkdir(exist_ok=True)
    (cal_dir / "camera_info.yaml").write_text(yaml_text(name, cal), encoding="utf-8")
    mp = session / "manifest.json"
    if mp.is_file():
        m = json.loads(mp.read_text(encoding="utf-8"))
        m.setdefault("calibration", {})["camera"] = {
            "status": "calibrated", "path": "calibration/camera_info.yaml",
            "source": f"checkerboard calibration, {name}",
            "notes": "distortion k3 is large: check reprojection error before trusting undistortion"
                     if abs(cal["dist"][-1]) > 1 else ""
        }
        tmp = mp.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(m, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        os.replace(tmp, mp)
        print(f"[ ok ] {session.name}: camera_info.yaml written, manifest updated")
    else:
        print(f"[ ok ] {session.name}: camera_info.yaml written (no manifest yet; run generate_manifest.py, "
              "then re-run this to set calibration status)")
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", type=Path, help="a session folder, or with --all the sessions root")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--camera", default=DEFAULT_CAMERA, choices=sorted(CAMERAS),
                    help=f"camera profile to install (default: {DEFAULT_CAMERA})")
    a = ap.parse_args()

    name = a.camera
    cal = CAMERAS[name]

    master = Path(__file__).resolve().parent / "calibration" / f"{name}.yaml"
    if not a.dry_run:
        master.parent.mkdir(exist_ok=True)
        master.write_text(yaml_text(name, cal), encoding="utf-8")

    targets = [p for p in sorted(a.path.iterdir()) if p.is_dir()] if a.all else [a.path]
    n = sum(install(t.resolve(), a.dry_run, name, cal) for t in targets)
    print(f"{n}/{len(targets)} session(s) handled")
    return 0


if __name__ == "__main__":
    sys.exit(main())