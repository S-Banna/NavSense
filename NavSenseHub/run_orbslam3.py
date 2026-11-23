
#!/usr/bin/env python3
"""Run the local ORB-SLAM3 RGB-D executable for one NavSense session."""

import argparse
import json
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
EXECUTABLE = REPO / "experiments/orbslam3_rgbd_baseline/build/test_sample1_rgbd"


def now():
    return datetime.now(timezone.utc).isoformat()


def stage_update(session, status, **extra):
    manifest_path = session / "manifest.json"

    if not manifest_path.is_file():
        raise FileNotFoundError(f"Missing manifest: {manifest_path}")

    with manifest_path.open(encoding="utf-8") as stream:
        manifest = json.load(stream)

    stages = manifest.setdefault("stages", {})
    stage = stages.setdefault("orbslam3", {})
    stage.update({"status": status, **extra})

    tmp_path = manifest_path.with_name("manifest.json.tmp")

    with tmp_path.open("w", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2)
        stream.write("\n")

    os.replace(tmp_path, manifest_path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", required=True, type=Path)
    args = parser.parse_args()

    session = args.session.resolve()

    if not session.is_dir():
        parser.error(f"Session directory not found: {session}")

    if not EXECUTABLE.is_file():
        parser.error(f"SLAM executable not built: {EXECUTABLE}")

    if not (session / "manifest.json").is_file():
        parser.error("Session has no manifest.json")

    if not (session / "mav0/cam0/data").is_dir():
        parser.error("Session has no RGB image folder")

    if not (session / "depth").is_dir():
        parser.error("Session has no depth folder")

    logs = session / "logs"
    logs.mkdir(exist_ok=True)

    log_path = logs / "orbslam3.log"

    stage_update(
        session,
        "running",
        started_utc=now(),
        inputs=["mav0/cam0/data", "depth"],
        log="logs/orbslam3.log"
    )

    try:
        lib_root = Path.home() / "opt/orbslam-deps"

        lib_paths = [
            lib_root / "openjph-0.31-compat/usr/lib",
            lib_root / "openexr-3.4-compat/usr/lib",
            lib_root / "opencv-4.6.0-install/lib",
        ]

        env = os.environ.copy()
        env["LD_LIBRARY_PATH"] = ":".join(
            [str(p) for p in lib_paths] +
            [env.get("LD_LIBRARY_PATH", "")]
        )

        with log_path.open("w", encoding="utf-8") as log:
            with subprocess.Popen(
                [str(EXECUTABLE), str(session)],
                cwd=str(REPO),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=env
            ) as process:
                for line in process.stdout:
                    print(line, end="", flush=True)
                    log.write(line)
                    log.flush()

                code = process.wait()

        output = session / "slam/orbslam3"
        required = output / "trajectory_tum.txt"
        atlas = output / "atlas.osa"

        if code == 0 and required.is_file() and atlas.is_file():
            stage_update(
                session,
                "done",
                finished_utc=now(),
                outputs=[
                    "slam/orbslam3/trajectory_tum.txt",
                    "slam/orbslam3/trajectory_meta.json",
                    "slam/orbslam3/atlas.osa"
                ]
            )
            return 0

        error = (
            f"SLAM exit code {code}; "
            f"trajectory exists={required.is_file()}; "
            f"atlas exists={atlas.is_file()}"
        )

    except Exception as exc:
        error = str(exc)

    stage_update(
        session,
        "failed",
        finished_utc=now(),
        error=error
    )

    print(error, file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
