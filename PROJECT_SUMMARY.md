# NavSense: project summary (baseline for the full documentation)

Status: written from the working sessions so far. Items marked **PLACEHOLDER** are intentionally left to be later filled.

---------------------------------------------------------------------------------------------------------------------

## 1. Goal and big picture

Final-year project: a small **robot car** carries an Android **phone** (camera + IMU) and an **Arduino** (motors,
wheel encoders). The phone streams sensor data to a laptop, where SLAM builds a map and localises the robot in it;
the longer-term target is coordinate/language-driven navigation (Nav2, plus a VLM for "go to the laboratory").
Depth comes from a monocular depth network (**Depth Anything 3, DA3**) run offline, because it is too slow for the
live loop (~5 FPS).

Two SLAM tracks are compared: **RTAB-Map** (ROS 2, on one Windows laptop under WSL2) and **ORB-SLAM3**
(a separate Linux machine; see section 10). NavSenseHub will be a single, demo-able "hub" app that ties everything
together (section 14).

```
 phone (Android app) -- Wi-Fi/TCP --> laptop receiver --> session folder         --> DA3 (offline depth)
   ^ camera, IMU                       (Windows script   (see SESSION_CONTRACT)  --> RTAB-Map (ROS 2, WSL2)
   | USB-OTG                           or ROS 2 bridge)                          --> ORB-SLAM3 (PLACEHOLDER)
   |                                          |
 Arduino + L298N + 2 encoders  <---- car commands (w/a/d/stop) via the phone
```

Documents in this folder: `PROJECT_SUMMARY.md` (this), `SESSION_CONTRACT.md` (data/stage contract),
`manifest.schema.json` + `manifest.example.json`, and `doctor.py` (+ `hub_config.example.json`).

---------------------------------------------------------------------------------------------------------------------

## 2. Hardware

| part | notes |
|---|---|
| Robot | L298N dual H-bridge, two DC motors (differential, on/off only: forward-only commands) |
| Arduino | Uno/Nano-class (ATmega328P), CH340 USB-serial chip (shows as "ch34x") |
| Motor pins | IN1=5, IN2=6 (left side), IN3=9, IN4=10 (right side) |
| Encoders | single-channel (one OUT pin each): right on D2 (hardware interrupt INT0), left on D3 (hardware interrupt) |
| Link to phone | USB-OTG hub/adapter, library `usb-serial-for-android` 3.8.0, 9600 baud |
| Power | motors must have their own battery; common GND with the Arduino; never power motors from the phone |

**Firmware (`car_encoder.ino`)**: reads one character at a time, no newline: `w` both sides forward, `a` left side
only, `d` right side only, **any other character (including a newline) stops everything**. Note `a` drives only the
left wheels, so the car turns *right*; `d` turns it *left*. Every command prints a reply line. Encoder rising edges
are counted by interrupts (200 us debounce) and printed at 10 Hz as `ENC,<arduino_millis>,<left>,<right>`.

---------------------------------------------------------------------------------------------------------------------

## 3. Phone app ("NavSense Logger", Kotlin)

History: the first version (CameraX + software YUV->JPEG, one TCP write per IMU sample) delivered ~3 fps at 1080p
and ~11 fps at 640x480. Cause: the phone, not Wi-Fi. Replaced with a hardware pipeline.

Current design:
- **Video**: Camera2 -> `MediaCodec` hardware H.264 encoder (Surface input) -> Annex-B packets. 1280x720 @ 30 fps,
  CBR ~4 Mbps (0.15 bits/pixel), I-frame every second, no B-frames, SPS/PPS repeated on keyframes. The size picker
  takes the largest size that both camera and encoder support at 30 fps under a configurable cap (`Cfg.MAX_W/H`).
  Video stabilisation and OIS are forced off (they warp the image); focus can be locked (`Cfg.LOCK_FOCUS`,
  `FOCUS_DIOPTERS`) so calibration stays valid.
- **Timestamps**: the encoder's presentation time can be on a different clock base from `elapsedRealtimeNanos`
  (observed ~3 days offset on a MediaTek phone). The app detects this and corrects it (`cam_clock` message).
- **IMU**: accel + gyro (magnetometer optional/off) requested at 200 Hz, delivered ~245-250 Hz, sent in small
  batches (2 samples per sensor per packet, ~100 packets/s) using each sensor event's own timestamp.
- **Network**: ONE writer thread with a queue (the original design had video and IMU fighting over one lock).
  If the video backlog exceeds `MAX_VIDEO_BACKLOG_BYTES` (2 MB) it drops P-frames until the next keyframe; this
  leaves ~1 s gaps in the data (bad for SLAM). IMU/control messages are never dropped.
- **Protocol** (unchanged since v0.2): `[u32 header_len][JSON header][payload]`. Message types phone->laptop:
  `hello`, `sync_resp`, `video` (+payload), `imu_batch`, `encoder`, `cam_clock`, `video_info`, `camera_chars`,
  `imu_info`, `car_reply`. Laptop->phone: `sync_req`, `start`, `cmd`.
- **Handshake**: laptop sends 16 `sync_req` rounds, takes the lowest-RTT one (mono + wall offsets), then sends
  `start`; the phone opens camera and sensors only after that.
- **Car link (`CarLink.kt`)**: finds the USB serial device (retries every 2 s), asks permission, opens 9600 baud,
  asserts DTR/RTS. Commands: single byte `w`/`a`/`d`/`s`, written on change and refreshed every 500 ms while moving.
  Safety: the laptop sends a heartbeat 10x/s; **the phone stops the car after 600 ms without heartbeats**, and on
  disconnect. Incoming serial bytes are reassembled into lines; `ENC,...` lines become `encoder` messages stamped
  with the phone clock; other lines are forwarded as `car_reply` (repeats suppressed).
- **Reported once per run** (for diagnosis): `imu_info` (sensor names/vendors/rates) and `camera_chars` (any
  calibration data the phone itself exposes; budget phones usually report none).
- Measured performance (university Wi-Fi): 30 fps, camera->encoder ~100 ms (camera/ISP pipeline, not network),
  Wi-Fi ~9-12 ms, decode ~2-5 ms, end-to-end ~120-130 ms. Home Wi-Fi was poor (150-500 ms network, stalls).

---------------------------------------------------------------------------------------------------------------------

## 4. Phones tried

| phone | result |
|---|---|
| Samsung A24 (MediaTek G99) | real gyro; USB data path faulty on the unit (no data even after cleaning) -> can't drive the Arduino |
| Samsung A15 4G (SM-A155F) | USB works with the Arduino; **no gyroscope** (IMU stream empty); good for testing the car/encoder side |
| Oppo A5 Pro | **fake gyro** (exactly 10 ms timestamps, two axes exactly 0 while the accelerometer shows rotation) |
| Tecno CLA6 (Camon 30S Pro) | spec lists a gyro and USB-C OTG; used for the recorded datasets |

---------------------------------------------------------------------------------------------------------------------

## 5. Windows receiver (recording) - `navsense_receiver.py` v0.4

Runs on the lead's Windows laptop. `python navsense_receiver.py --no-display` listens on port 8765.
- Decodes H.264 (PyAV) and writes them
  on background threads as `mav0/cam0/data/<capture_ns>.jpg` (PNG optional). Frame names carry the capture time.
- Saves everything in the layout of `SESSION_CONTRACT.md` (raw video + per-packet index, `imu.jsonl`,
  `encoder.csv`, `events.jsonl`, `clock_sync.json`, `session_info.json`).
- **Live IMU health check** at ~6 s: prints accel/gyro rates, or a loud warning for no gyro / fake gyro / dead axes.
- **Keyboard drive**: `w`/`a`/`d` go, **any other key stops**; a heartbeat thread keeps the phone's watchdog fed.
- **Export** at session end (or `--export <folder>`): EuRoC-style `mav0/imu0/data.csv` (accel interpolated onto gyro
  timestamps), `mav0/cam0/data.csv`, `timestamps.txt`, `orbslam3_phone.yaml` (settings template with placeholder
  calibration), and `sync_report.txt` (frame gaps, IMU samples per frame, gravity check, fake-gyro heuristics).
  Options: `--timeshift-ms`, `--static-seconds`, `--noise-scale`.
- `extract_frames.py <session>` re-creates all frames from `video.h264` + `video.jsonl` (same names/timestamps).
- Bugs found on the way: encoder field names (`ard_ms`/`t_ns`) mismatched in one copy (crashed on the first encoder
  line); PNG saving in the network thread would stall the stream (hence the writer threads).

---------------------------------------------------------------------------------------------------------------------

## 6. ROS 2 on WSL2

Environment: Windows 11 + WSL2, Ubuntu 24.04, **ROS 2 Jazzy** (`ros-jazzy-desktop`, `ros-dev-tools`, installed via the
`ros2-apt-source` package). Workspace `~/navsense_ws`.

Networking (so the phone can reach WSL): `%UserProfile%\.wslconfig` -> `[wsl2] networkingMode=mirrored`, plus
inbound rules for TCP 8765 in Windows Firewall and in the Hyper-V firewall (display name `NavSense 8765`), then
`wsl --shutdown`. Verified with a Python HTTP server reachable from the phone.

**`navsense_bridge` package** (Python, ament_python; install from `navsense_bridge.tar.gz`, `colcon build`):
- Executable `bridge`: TCP server for the phone (replaces the Windows receiver for live use). Publishes
  `/camera/image_raw` (portrait, `bgr8`/`mono8`, only while subscribed), `/camera/camera_info` (only with a real
  calibration file), `/imu/data_raw` (gyro + accel merged onto gyro stamps), `/wheel_ticks` (JointState). Subscribes
  `/cmd_vel` and maps it to `w`/`a`/`d`/stop with a 0.5 s timeout. Logs raw accel/gyro rates and **errors loudly if
  there are accelerometer samples but no gyro samples**. Phone info messages go to `navsense_phone_info.jsonl`.
- Executable `replay`: publishes a recorded folder pair (`rgb_dir`, `depth_dir`, frames paired by number) as
  `/camera/image_raw`, `/camera/depth/image_raw` (32FC1 metres), `/camera/camera_info`, plus the static transform
  `base_link -> camera`. Intrinsics come from an ORB-SLAM3 settings yaml or parameters (never invented);
  `inspect_only:=true` reports depth encoding/plausibility; `stride`, `scale`, `speed`, `fps` control load.
  **Planned**: extend it to also replay IMU and encoder data from a recorded session.
- Manual driving: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p key_timeout:=0.6` (hold a key;
  ~0.5 s stop latency by design; a sticky-key driver is a possible improvement).
- Camera calibration (not yet done): `ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025
  --no-service-check --ros-args -r image:=/camera/image_raw` (the flag is needed because the bridge has no
  `set_camera_info` service; use *Save*, not *Commit*).

Gotchas: **if `ros2 topic list` hangs or nodes can't see each other, run `ros2 daemon stop; ros2 daemon start`**
(this cost hours once; RTAB-Map's "did not receive data" warnings were also just this). RTAB-Map's `rtabmap_viz`
prints a harmless Qt "wrong permissions on runtime directory" warning under WSLg.

---------------------------------------------------------------------------------------------------------------------

## 7. RTAB-Map offline experiment (so far)

- Input: two recordings as folders `rgb/` (jpg) and `depth/` (16-bit PNG, millimetres) named `*_frame_NNNN`:
  `output_orbslam_metric3` (corridor + room, 1035 frames) and `output_orbslam_metric4` (second pass, ~1028 frames).
  DEPRECATED: Images are 720x1280 portrait; intrinsics from the ORB-SLAM3 settings `camera.yaml` (fx 1042.0, fy 1040.3,
  cx 358.9, cy 646.4, 30 fps); typical depth ~5.7 m (plausible indoors).
- Mapping: `ros2 launch rtabmap_launch rtabmap.launch.py rtabmap_args:="--delete_db_on_start" rgb_topic:=...
  depth_topic:=... camera_info_topic:=... frame_id:=base_link approx_sync:=false qos:=1 database_path:=~/maps/<name>.db
  rtabmap_viz:=true`. Localisation: same without `--delete_db_on_start`, plus `localization:=true`.
- Result: the pipeline ran, localisation mode found loop closures and loaded the old map, but the map looked
  odd. No conclusion yet: those recordings have no wheel/IMU data (RTAB-Map used its own RGB-D visual odometry)
  and the monocular depth scale may drift between frames. A fair comparison needs new recordings with IMU +
  encoders in a good environment.

---------------------------------------------------------------------------------------------------------------------

## 8. DA3 (Depth Anything 3)

- **Environment:** Windows 11, using a Python environment with the `depth_anything_3` package installed. Repository at `C:\FYP\Depth-Anything-3\` (ByteDance-Seed/Depth-Anything-3). This is separate from the ROS 2 Jazzy environment running under WSL2.
- **Python API:** `from depth_anything_3.api import DepthAnything3`. CLI: `da3`.
- **Model:** Hugging Face DA3 model (`depth-anything/DA3...`). Exact model variant: DA3METRIC-LARGE.
- **GPU**: NVIDIA RTX 4050 Laptop GPU, CUDA enabled.
- **Depth mode:** Metric depth variant, using focal length. Depth conversion: `depth = focal_px × output / 300`.
- **Usage:** Runs offline on recorded RGB frames to generate depth maps, which are subsequently paired with RGB frames for RTAB-Map experiments. Not used in the live loop because inference is too slow (approximately 5 FPS).

---------------------------------------------------------------------------------------------------------------------

## 9. Calibration status (nothing is calibrated yet)

| item | state | plan |
|---|---|---|
| Camera intrinsics (1280x720) | calibrated against two phones | checkerboard with `camera_calibration`, focus locked |
| Camera-IMU rotation | landscape matrices (see table 2.2 in session contract), status still derived, untested | verify |
| Camera-IMU translation + time offset | unknown (zeros) | Kalibr, or online estimation inside the VIO |
| Wheel | calibrated by arithmetic, still needs the 2 m taped test | m_per_pulse = 0.0102102, pulses_per_m = 97.94, track_width_m = 0.135; track width 135 mm is geometric; effective width during pivot is larger (wheel scrub), should measure before trusting it for turns |
| IMU noise | EuRoC defaults x5 | Allan variance, or static recording (`--static-seconds`) |

---------------------------------------------------------------------------------------------------------------------

## 10. ORB-SLAM3 Integration

ORB-SLAM3 is integrated into NavSenseHub for offline RGB-D mapping using recorded phone RGB images and DA3-generated depth maps.

### Machine and Build

- **OS:** EndeavourOS Linux (Arch-based).
- **ORB-SLAM3 source:** `~/opt/orbslam-deps/ORB_SLAM3`.
- **Version/commit:** Not recorded.
- **Build:** CMake using OpenCV, Eigen3, Pangolin, Boost Serialization, and required shared libraries.
- **ROS wrapper:** None; the current implementation uses a standalone C++ executable.
- **Executable:** `orbslam/build/test_sample1_rgbd`.
- **Readiness checks:** `NavSenseHub/doctor.py` checks the executable, session runner, settings YAML, ORB vocabulary, and shared libraries.

### Implemented Mode

**RGB-D mapping** is implemented and tested. Monocular and monocular-inertial modes are not integrated into the current session-based pipeline.

Run directly:

```bash
./orbslam/build/test_sample1_rgbd \
    "./NavSenseLogger/sessions/20261119_115854"
```

Or through the session runner:

```bash
python NavSenseHub/run_orbslam3.py \
    --session "./NavSenseLogger/sessions/20261119_115854"
```

ORB-SLAM3 can also be launched using the **Run** button for any ready session in NavSenseHub.

### Settings and Inputs

- **Settings YAML:** `orbslam/orbslam3_tecno.yaml`.
- **Camera:** Tecno CLA6, 720 × 1280, newer recordings use Infinix Note 30 Pro, 1280 × 720.
- **Calibration source:** `NavSenseLogger/calibration/tecno_cla6_720x1280.yaml`.
- **RGB input:** `mav0/cam0/data/<timestamp>.jpg`.
- **Depth input:** `depth/<timestamp>.png`, 16-bit, in millimetres.
- **Depth source:** Depth Anything 3.
- **Depth factor:** `RGBD.DepthMapFactor: 1000.0`.
- **Timestamps:** Extracted from image filenames.
- **IMU noise and Tbc:** Placeholder values, not used by RGB-D mode.

### Outputs

Each session stores its own results:

```text
<session>/
├── slam/orbslam3/
│   ├── atlas.osa
│   ├── trajectory_tum.txt
│   ├── keyframe_trajectory_tum.txt
│   ├── trajectory_meta.json
│   └── orbslam3_run.yaml
└── logs/
    └── orbslam3.log
```

The trajectory uses TUM format (`timestamp tx ty tz qx qy qz qw`). Metadata identifies the camera coordinate frame, metric scale, mapping mode, and processing statistics.

The Python runner updates the session's `manifest.json` with the ORB-SLAM3 execution status and output paths.

### Viewer and Hub

- Pangolin displays the camera trajectory and reconstructed map during execution.
- Atlas and trajectory outputs are saved after successful completion.
- NavSenseHub launches the Python runner with the selected session path and displays live logs.
- Doctor reports `orbslam3: ready` or `orbslam3: not ready`.
- The Run button is enabled for sessions with the required RGB images, depth maps, and manifest.
- `hub_config.json` supports local configuration; the ORB-SLAM3 Doctor check detects the current installation automatically.

### Known Issues

- Tracking can become unstable, causing relocalization attempts and eventual tracking loss.
- Reconstructed maps may become distorted before tracking fails.
- Earlier experimental localization changes caused repeated map resets; the previous ORB-SLAM3 source backups were restored.
- Tracking statistics in `trajectory_meta.json` are not fully reliable.
- Cancelling a Hub job may leave its child C++ process running on Linux.

**Current status:** Offline RGB-D mapping, per-session outputs, Pangolin visualization, Doctor readiness checks, and NavSenseHub Run-button integration are implemented. Tracking accuracy and reliability remain unresolved.

## 11. VLM / language grounding

Two approaches explored, both using recorded RGB frames as the retrieval corpus:

**Semantic place retrieval (OpenCLIP):** Every 10th frame is embedded with OpenCLIP. A text query
("go to the kitchen") is embedded in the same space; cosine search finds the best-matching frame.
The pose stored for that frame becomes the Nav2 goal.

**Visual re-localisation (DINOv2):** Starting from an unknown position, the current live camera view
is matched against the stored frame embeddings using DINOv2 features. The best match yields an
estimated current pose — a form of appearance-based dead-reckoning localisation.

Both paths converge on the same output: a pose in the map frame retrieved from a matched frame.
Open: integrating the goal pose into Nav2, handling query ambiguity (multiple plausible matches),
and deciding whether embeddings are pre-computed offline or on-demand.

---------------------------------------------------------------------------------------------------------------------

## 12. Lessons learned (short list)

1. The original slowness was the phone's software JPEG path, not Wi-Fi; hardware H.264 fixed it.
2. Timestamps are the whole game for SLAM: one clock (phone `elapsedRealtime`), per-sensor timestamps, accel and
   gyro merged on gyro stamps, camera timestamp base verified per phone.
3. Check the gyro on every phone within seconds (missing on A15 4G, fake on Oppo A5 Pro).
4. Gaps >= ~1 s in the image stream make ORB-SLAM3 reset its map (message "Timestamp jump"); the phone's
   drop-until-keyframe behaviour creates such gaps when Wi-Fi stalls.
5. An Arduino newline is a stop command; commands must be single bytes.
6. Big images in Python/ROS: write files on background threads; publish only when someone subscribes.
7. ROS 2 daemon problems look like SLAM problems. Restart the daemon first.
8. Spec sheets lie about gyros; USB ports die; keep a second phone.

## 13. Open backlog (suggested order)

1. Calibration: camera intrinsics, wheel parameters, camera-IMU (section 9).
2. Run DA3 on the new good recording; map it with RTAB-Map; compare with ORB-SLAM3 on the same session.
3. Extend `replay` to publish IMU + encoders from a session; add wheel odometry (`/odom`) and `robot_localization`.
4. Fill the placeholders (ORB-SLAM3, VLM) and the DA3 environment notes.
5. Hub: manifest generator, stage runners, live ingest -> RTAB-Map, dashboards (PlotJuggler/Foxglove/rqt).
6. Live RTAB-Map + Nav2 (stretch: the car is forward-only and bang-bang, which Nav2 does not expect).

## 14. Planned hub and decisions so far

Requested: one app/hub that (1) loads and prepares offline data, (2) runs DA3, (3) routes to ORB-SLAM3 or RTAB-Map,
(4) wraps their offline visualisers with start/save/stop, (5) ingests the robot's live stream for ROS 2,
(6) feeds live RTAB-Map + Nav2, (7) shows streams/graphs/replay, (8) localises in real time, (9) simulates "live"
from a recording.

Decisions: files + `manifest.json` are the interface between sub-apps (`SESSION_CONTRACT.md`); each machine runs
`doctor.py` so unavailable stages are greyed out and cached results are used; "simulated live" (9) is the default
demo path because live demos fail (Wi-Fi, USB, throttling); live ORB-SLAM3 is skipped (offline benchmark only);
existing tools are wrapped, not rebuilt; a calibration stage and an evaluation stage (trajectory comparison with
`evo`) are added to the plan.

Suggested milestones: (1) contract + validator + fixture dataset, (2) one dataset end-to-end by hand (prepare -> DA3 ->
RTAB-Map -> localise), (3) CLI hub, then a simple web UI, (4) simulated-live + dashboards, (5) live robot -> RTAB-Map,
(6) Nav2.

## 15. File index (what exists and where)

| file | what | owner |
|---|---|---|
| `MainActivity.kt`, `CarLink.kt` | Android app | lead |
| `car_encoder.ino` | Arduino sketch (motors + 2 encoders) | lead |
| `navsense_receiver.py`, `extract_frames.py` | Windows recorder, export, frame re-extraction | lead |
| `navsense_bridge.tar.gz` | ROS 2 package (`bridge`, `replay`) | lead |
| `APP_PATCH.md` | app patch notes (encoder wiring, imu/camera info, focus lock) | lead |
| `SESSION_CONTRACT.md`, `manifest.*` | the contract | all |
| `doctor.py`, `hub_config.example.json` | per-machine readiness check | all (ORB-SLAM3 + VLM blocks) |

## 16. Glossary

**Session**: one recording folder. **Stage**: one processing step (DA3, RTAB-Map, ...). **Manifest**: `manifest.json`,
the index of a session. **Contract**: the rules every stage follows. **EuRoC format**: the dataset layout ORB-SLAM3's
examples read. **VIO**: visual-inertial odometry (needs a real gyro). **Doctor**: the readiness check. **Phone clock**:
`SystemClock.elapsedRealtimeNanos()` on the phone. **TUM format**: `timestamp x y z qx qy qz qw` trajectory text file.
