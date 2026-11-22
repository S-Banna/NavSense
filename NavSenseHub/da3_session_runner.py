
from __future__ import annotations

import argparse
import importlib.metadata
import json
import os
import platform
import shutil
import socket
import sys
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import torch
import yaml
from depth_anything_3.api import DepthAnything3 #type: ignore


DEFAULT_MODEL = "depth-anything/DA3METRIC-LARGE"
IMAGE_EXTS = {".jpg", ".jpeg", ".png"}


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


class Tee:
    """Write console output to both the terminal and the stage log."""

    def __init__(self, *streams):
        self.streams = streams

    def write(self, text):
        for stream in self.streams:
            stream.write(text)
            stream.flush()

    def flush(self):
        for stream in self.streams:
            stream.flush()


def atomic_json(path: Path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    os.replace(tmp, path)


def read_manifest(session: Path):
    path = session / "manifest.json"
    if not path.is_file():
        raise FileNotFoundError(f"Missing manifest: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def update_manifest(session: Path, update_fn):
    """Re-read immediately before updating, preserving unrelated stage entries."""
    path = session / "manifest.json"
    manifest = read_manifest(session)
    update_fn(manifest)
    atomic_json(path, manifest)


def get_frames(session: Path, manifest):
    rgb = manifest.get("streams", {}).get("rgb", {})

    if not rgb.get("present"):
        raise RuntimeError("manifest streams.rgb.present is not true")

    relative = rgb.get("path")
    if not relative:
        raise RuntimeError("manifest streams.rgb.path is missing")

    frame_dir = session / relative
    if not frame_dir.is_dir():
        raise FileNotFoundError(f"RGB frame directory does not exist: {frame_dir}")

    frames = []
    for path in frame_dir.iterdir():
        if not path.is_file() or path.suffix.lower() not in IMAGE_EXTS:
            continue

        try:
            timestamp_ns = int(path.stem)
        except ValueError:
            raise RuntimeError(
                f"Frame filename must be a nanosecond timestamp: {path.name}"
            )

        frames.append((timestamp_ns, path))

    frames.sort(key=lambda item: item[0])

    if not frames:
        raise RuntimeError(f"No RGB frames found in {frame_dir}")

    timestamps = [item[0] for item in frames]
    if len(timestamps) != len(set(timestamps)):
        raise RuntimeError("Duplicate RGB frame timestamps found")

    return frame_dir, frames


def read_camera_info(path: Path):
    """Read ROS camera_info YAML and return fx, fy, cx, cy, width, height."""
    data = yaml.safe_load(path.read_text(encoding="utf-8"))

    matrix = data.get("camera_matrix", {})
    values = matrix.get("data") if isinstance(matrix, dict) else None

    if not values or len(values) != 9:
        raise RuntimeError(f"Invalid camera_matrix in {path}")

    fx, fy = float(values[0]), float(values[4])
    cx, cy = float(values[2]), float(values[5])

    width = int(data.get("image_width", 0))
    height = int(data.get("image_height", 0))

    if fx <= 0 or fy <= 0:
        raise RuntimeError(f"Invalid focal lengths in {path}")

    return fx, fy, cx, cy, width, height


def read_orbslam_intrinsics(path: Path):
    """Fallback for the existing OpenCV-style orbslam3_phone.yaml."""
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Could not open calibration fallback: {path}")

    try:
        def number(key):
            node = fs.getNode(key)
            if node.empty():
                raise RuntimeError(f"Missing {key} in {path}")
            return float(node.real())

        fx = number("Camera1.fx")
        fy = number("Camera1.fy")
        cx = number("Camera1.cx")
        cy = number("Camera1.cy")

        width = int(number("Camera.width"))
        height = int(number("Camera.height"))

        return fx, fy, cx, cy, width, height
    finally:
        fs.release()


def get_intrinsics(session: Path, manifest):
    calibration = manifest.get("calibration", {}).get("camera", {})
    calibration_path = calibration.get("path", "calibration/camera_info.yaml")
    camera_file = session / calibration_path

    if camera_file.is_file():
        values = read_camera_info(camera_file)
        source = f"{calibration_path} (status={calibration.get('status', 'unknown')})"
        return values, source

    # Explicit fallback to the existing ORB-SLAM3 settings template.
    fallback = session / "orbslam3_phone.yaml"
    if fallback.is_file():
        values = read_orbslam_intrinsics(fallback)
        source = "orbslam3_phone.yaml (fallback; calibration file missing)"
        return values, source

    raise FileNotFoundError(
        "No usable camera intrinsics. Expected "
        f"{camera_file} or {fallback}"
    )


def validate_inputs(session: Path, manifest):
    if manifest.get("contract_version") != "1.0":
        raise RuntimeError(
            f"Unsupported session contract: {manifest.get('contract_version')}"
        )

    frame_dir, frames = get_frames(session, manifest)
    intrinsics, intrinsics_source = get_intrinsics(session, manifest)

    fx, fy, cx, cy, width, height = intrinsics

    first = cv2.imread(str(frames[0][1]), cv2.IMREAD_COLOR)
    if first is None:
        raise RuntimeError(f"Could not read frame: {frames[0][1]}")

    actual_height, actual_width = first.shape[:2]

    if (width, height) != (actual_width, actual_height):
        raise RuntimeError(
            f"Calibration resolution is {width}x{height}, but RGB frames are "
            f"{actual_width}x{actual_height}. Calibration must match the "
            "saved portrait images."
        )

    # Check a few more frames before starting expensive inference.
    for _, path in frames[1:5]:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"Could not read frame: {path}")
        if image.shape[:2] != (actual_height, actual_width):
            raise RuntimeError(f"Inconsistent frame dimensions: {path}")

    return {
        "frame_dir": frame_dir,
        "frames": frames,
        "fx": fx,
        "fy": fy,
        "cx": cx,
        "cy": cy,
        "width": width,
        "height": height,
        "focal_px": (fx + fy) / 2.0,
        "intrinsics_source": intrinsics_source,
    }


def update_stage(session: Path, status: str, **fields):
    def apply(manifest):
        stages = manifest.setdefault("stages", {})
        stage = stages.setdefault("depth_da3", {})
        stage["status"] = status
        stage.update(fields)

    update_manifest(session, apply)


def main():
    parser = argparse.ArgumentParser(description="NavSense DA3 session stage")
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument("--params", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    session = args.session.resolve()
    if not session.is_dir():
        raise FileNotFoundError(f"Session folder does not exist: {session}")

    params = {}
    if args.params:
        params = json.loads(args.params.read_text(encoding="utf-8"))

    model_id = params.get("model", DEFAULT_MODEL)
    batch_size = int(params.get("batch_size", 1))
    device_name = params.get("device", "auto")

    if batch_size < 1:
        raise ValueError("batch_size must be at least 1")

    manifest = read_manifest(session)
    inputs = validate_inputs(session, manifest)

    frames = inputs["frames"]
    output_dir = session / "depth"
    metadata_path = output_dir / "depth_meta.json"
    log_path = session / "logs" / "depth_da3.log"

    print(f"Session: {session}")
    print(f"Frames: {len(frames)}")
    print(f"Image size: {inputs['width']}x{inputs['height']}")
    print(f"Intrinsics: {inputs['intrinsics_source']}")
    print(f"Focal length used: {inputs['focal_px']:.4f} px")
    print(f"Model: {model_id}")
    print(f"Batch size: {batch_size}")

    if args.dry_run:
        print("\nDry run successful. No files or manifest entries were changed.")
        print(f"Would write depth PNGs to: {output_dir}")
        print(f"Would write metadata to: {metadata_path}")
        return 0

    output_dir.mkdir(parents=True, exist_ok=True)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    original_stdout, original_stderr = sys.stdout, sys.stderr

    with log_path.open("a", encoding="utf-8") as log:
        sys.stdout = Tee(original_stdout, log)
        sys.stderr = Tee(original_stderr, log)

        started = utc_now()
        start_time = time.perf_counter()

        try:
            update_stage(
                session,
                "running",
                machine={
                    "hostname": socket.gethostname(),
                    "os": platform.platform(),
                },
                started_utc=started,
                finished_utc=None,
                tool={
                    "name": "Depth-Anything-3",
                    "version": importlib.metadata.version("depth-anything-3"),
                },
                params={
                    "model": model_id,
                    "batch_size": batch_size,
                    "focal_px_used": inputs["focal_px"],
                    "stride": 1,
                },
                inputs=[
                    str(Path(manifest["streams"]["rgb"]["path"]).as_posix()),
                    inputs["intrinsics_source"],
                ],
                outputs=["depth/", "depth/depth_meta.json"],
                log="logs/depth_da3.log",
                error=None,
            )

            if device_name == "auto":
                device = torch.device(
                    "cuda" if torch.cuda.is_available() else "cpu"
                )
            else:
                device = torch.device(device_name)

            if device.type == "cuda" and not torch.cuda.is_available():
                raise RuntimeError("CUDA was requested but is not available")

            gpu = (
                torch.cuda.get_device_name(device)
                if device.type == "cuda"
                else "CPU"
            )

            print(f"\nLoading {model_id} on {device} ({gpu})...")
            model = DepthAnything3.from_pretrained(model_id).to(device=device)
            model.eval()

            focal = inputs["focal_px"]
            total = len(frames)

            print(f"Processing {total} frames...")

            for start in range(0, total, batch_size):
                batch = frames[start:start + batch_size]
                paths = [str(path) for _, path in batch]

                with torch.inference_mode():
                    prediction = model.inference(paths)

                raw_depth = np.asarray(prediction.depth, dtype=np.float32)

                if raw_depth.ndim != 3 or raw_depth.shape[0] != len(batch):
                    raise RuntimeError(
                        f"Unexpected DA3 output shape: {raw_depth.shape}"
                    )

                for index, (timestamp_ns, source_path) in enumerate(batch):
                    depth = raw_depth[index]

                    if depth.shape != (inputs["height"], inputs["width"]):
                        depth = cv2.resize(
                            depth,
                            (inputs["width"], inputs["height"]),
                            interpolation=cv2.INTER_LINEAR,
                        )

                    # DA3METRIC-LARGE model-card conversion.
                    depth_m = focal * depth / 300.0
                    depth_m = np.nan_to_num(
                        depth_m, nan=0.0, posinf=0.0, neginf=0.0
                    )
                    depth_m[depth_m <= 0] = 0.0

                    depth_mm = np.clip(
                        np.rint(depth_m * 1000.0),
                        0,
                        65535,
                    ).astype(np.uint16)

                    destination = output_dir / f"{timestamp_ns}.png"

                    if not cv2.imwrite(str(destination), depth_mm):
                        raise RuntimeError(
                            f"Failed to write depth image: {destination}"
                        )

                done = min(start + len(batch), total)
                print(f"Processed {done}/{total}")

                del prediction, raw_depth
                if device.type == "cuda":
                    torch.cuda.empty_cache()

            runtime = time.perf_counter() - start_time

            metadata = {
                "model": model_id,
                "scale": "metres = png * 0.001",
                "focal_px_used": focal,
                "intrinsics_source": inputs["intrinsics_source"],
                "image_size": {
                    "width": inputs["width"],
                    "height": inputs["height"],
                },
                "frame_count": total,
                "created_utc": utc_now(),
                "runtime_s": round(runtime, 3),
                "gpu": gpu,
            }

            atomic_json(metadata_path, metadata)

            def update_depth_stream(manifest):
                manifest.setdefault("streams", {})["depth"] = {
                    "present": True,
                    "path": "depth",
                    "meta": "depth/depth_meta.json",
                    "count": total,
                }

            update_manifest(session, update_depth_stream)

            update_stage(
                session,
                "done",
                finished_utc=utc_now(),
                metrics={
                    "frame_count": total,
                    "runtime_s": round(runtime, 3),
                },
                error=None,
            )

            print("\nDA3 stage completed.")
            print(f"Depth images: {output_dir}")
            print(f"Metadata: {metadata_path}")
            print(f"Runtime: {runtime:.1f}s")
            return 0

        except Exception as exc:
            print("\nDA3 stage FAILED.")
            traceback.print_exc()

            try:
                update_stage(
                    session,
                    "failed",
                    finished_utc=utc_now(),
                    error=str(exc)[:2000],
                )
            except Exception:
                print("WARNING: Could not update manifest failure status.")
                traceback.print_exc()

            return 1

        finally:
            sys.stdout = original_stdout
            sys.stderr = original_stderr


if __name__ == "__main__":
    raise SystemExit(main())