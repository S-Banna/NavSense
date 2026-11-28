# NavSense Session Contract v1.0

**What this document is.** A *contract* is the agreement that lets separate programs, written by different
people and run on different laptops, work together without knowing about each other. Here, the contract says:
what a recorded **session** looks like on disk, what every number in it means, and how each processing
**stage** (DA3, RTAB-Map, ORB-SLAM3, ...) reads inputs and writes outputs. The `manifest.json` is the index file
inside every session folder that records what exists and which stages have run.

If you build or change a stage, this is the only thing you need to satisfy. You never need to run anybody
else's code.

Companion files: `manifest.schema.json` (formal shape), `manifest.example.json` (filled example),
`../navsense_hub/doctor.py` (per-machine readiness check), `PROJECT_SUMMARY.md` (how we got here).

---------------------------------------------------------------------------------------------------------------------

## 1. Principles

1. **A session is a folder.** Moving it = copying/zipping the folder. No database, no server.
2. **Raw data is immutable.** Stages only *add* files in their own folders and update their own entry in
   `manifest.json`. Nobody edits `imu.jsonl`, `encoder.csv`, the RGB frames, or the raw video.
3. **One clock.** Every timestamp is the phone's `elapsedRealtime` in integer nanoseconds (section 3.1).
4. **Files, not function calls.** A stage = "read these paths, write those paths, report status". It can run
   anywhere that has the files and the tool installed.
5. **Optional means optional.** A machine that cannot run a stage simply skips it. `doctor.py` says what a
   machine can run, and cached results from other machines are used instead.
6. **Placeholders are explicit.** Anything not yet decided is marked `placeholder` / `TODO` in the manifest and
   in this document, never silently assumed.

---------------------------------------------------------------------------------------------------------------------

## 2. Folder layout

`<session_id>` is `YYYYMMDD_HHMMSS` (created by the recorder). Paths are relative to the session folder.
"Made by" tells you which program writes the file.

```
<session_id>/
  manifest.json                    index + stage status                         made by: prepare, then every stage
  # ---- raw recording (immutable) ------------------------------------------------------------------------
  video.h264                       raw H.264 (Annex-B), all packets concatenated  receiver
  video.jsonl                      ONE line per packet incl. SPS/PPS, with file_offset   receiver
  imu.jsonl                        raw accel/gyro samples                         receiver
  encoder.csv                      wheel pulse counts from the Arduino            receiver
  events.jsonl                     phone/Arduino messages (video_info, imu_info, camera_chars, cam_clock, car replies)
  clock_sync.json                  phone-vs-laptop clock offsets at connect time  receiver
  session_meta.json                sensor_orientation, rotation_cw_deg, frames_fmt   receiver
  session_info.json                counts, IMU health verdict, durations          receiver
  # ---- derived by "prepare" (regenerable from the raw files) -------------------------------------------
  mav0/cam0/data/<ts_ns>.jpg|png   RGB frames, upright LANDSCAPE, named by capture time   receiver (live) or extract_frames.py
  mav0/cam0/data.csv               "#timestamp [ns],filename"                      prepare
  mav0/imu0/data.csv               merged 6-axis IMU, EuRoC column order           prepare
  timestamps.txt                   one frame timestamp (ns) per line               prepare
  orbslam3_phone.yaml              ORB-SLAM3-style settings TEMPLATE (placeholders for calibration)   prepare
  sync_report.txt                  gaps, IMU samples per frame, gyro sanity        prepare
  # ---- calibration (added by calibration tools; may be missing or placeholder) --------------------------
  calibration/camera_info.yaml     ROS camera_info (ost.yaml) for the saved portrait image size
  calibration/extrinsics.json      IMU<->camera rotation/translation + time offset
  calibration/wheel.json           encoder scale + track width
  # ---- stage outputs ----------------------------------------------------------------------------------
  depth/<ts_ns>.png                DA3 depth, 16-bit PNG, millimetres, 0 = invalid  depth_da3
  depth/depth_meta.json            model, scale, focal used, runtime               depth_da3
  slam/rtabmap/...                 map.db, trajectory_tum.txt, trajectory_meta.json, grid.pgm/.yaml   rtabmap_offline / rtabmap_live
  slam/orbslam3/...                PLACEHOLDER (teammate decides; must include trajectory_tum.txt + trajectory_meta.json)
  semantic/...                     PLACEHOLDER (VLM)
  eval/...                         FUTURE: metrics.json, plots
  logs/<stage>.log                 one log per stage run
```

A session may be **partial**: for example only `mav0/`, `imu.jsonl`, `encoder.csv` and `manifest.json` after a
transfer. The manifest's `streams.*.present` flags say what is really there.

---------------------------------------------------------------------------------------------------------------------

## 3. Conventions (read this before touching any number)

### 3.1 Time
- Unit: **integer nanoseconds**. Clock: the phone's `SystemClock.elapsedRealtimeNanos()` ("phone clock").
- Camera: capture time of the frame (start of exposure as reported by the camera HAL). The encoder's timestamps
  can be on a different base on some phones (we saw a 3-day offset on MediaTek); the app detects and corrects it
  and reports it as a `cam_clock` event. Frame file name = this timestamp.
- IMU: the sensor event's own timestamp (not the arrival time).
- Encoders: `t_ns` = phone clock when the serial line arrived at the phone (includes USB jitter);
  `arduino_ms` = the Arduino's `millis()` when it printed. Use `arduino_ms` deltas for exact spacing.
- `clock_sync.json` holds `mono_offset_ms` / `wall_offset_ms` = **phone minus laptop**, measured once at connect
  (lowest-RTT of 16 rounds; accuracy a few ms). Laptop wall time of a phone timestamp:
  `laptop_wall_s = (t_ns/1e6 - mono_offset_ms)/1e3`.
- ROS stamps: **live** = laptop wall time via the formula above. **Replay of a session** = replay start time +
  `(t_ns - time_base.t0_ns)/1e9`. Either way, relative timing between sensors is preserved exactly.
- JSON numbers carry these integers. Current values are ~5e15, safely below 2^53 (9.007e15); JavaScript tools
  must still use BigInt if a phone has been up for >100 days.

### 3.2 Coordinate frames
- **Image / camera frame(()) (OpenCV optical): x right, y down, z forward. Images are saved **upright landscape**
  1280×720 (back camera, phone landscape, top of phone to the robot's left, rotation_cw_deg = 0). All
  intrinsics refer to this saved landscape image.
- **IMU / body frame** (Android device frame, phone held landscape): x right, y up, z out of the screen.
- **Robot frame `base_link`** (ROS REP-105): x forward, y left, z up. **Map** frame: same convention.
- **Expected mounting**: phone in landscape, back camera facing forward.

  | transform | rotation matrix (rows) | note |
  |---|---|---|
  | base_link -> camera (optical) | `[[0,0,1],[-1,0,0],[0,-1,0]]` | = rpy(-90deg, 0, -90deg); quaternion (-0.5, 0.5, -0.5, 0.5) |
  | base_link -> imu (Android axes) | `[[0,0,-1],[0,1,0],[1,0,0]]` | |
  | imu -> camera (R_b_c for SLAM) | `[[0,-1,0],[-1,0,0],[0,0,-1]]` | consistent with the two rows above |

  rotation_cw_deg = 0 on this phone; r_body_cam(90, 0) gives the landscape camera -> body rotation. If a future phone needs a different rotation_cw_deg, r_body_cam computes the general case.

### 3.3 Units and sensors
- Gyro: rad/s. Accelerometer: m/s^2 **including gravity** (a still phone reads ~9.8 on one axis). Both in the
  Android device frame. `imu.jsonl` keeps accel and gyro as separate samples with their own timestamps
  (~245-250 Hz each when healthy).
- `mav0/imu0/data.csv` (EuRoC order `t,wx,wy,wz,ax,ay,az`): ONE row per **gyro** timestamp with the accelerometer
  linearly interpolated onto it. This is what SLAM tools need (they cannot take two unsynchronised streams).
- `gyro_verdict` in the manifest: `ok`, `fake` (software-generated: perfectly regular timestamps, zero axes),
  `missing` (no gyro samples), `unknown`. **A session with `fake`/`missing` is not usable for visual-inertial work.**
- Encoders: `left` and `right` are cumulative rising-edge counts since the Arduino booted (left = D3,
  right = D2). Single channel: **no direction information** (the car only drives forward). They reset when the
  Arduino resets (each time the phone app opens the USB port on Uno/Nano boards): a drop in the count means a reset.

### 3.4 Depth (stage `depth_da3`)
- `depth/<ts_ns>.png`: 16-bit PNG, **millimetres**, 0 = invalid, same pixel grid as the RGB frame with the same
  timestamp. ROS replay converts to 32-bit float metres.
- DA3 metric models need the focal length: metric depth = focal_px x network_output / 300 (focal = mean of fx, fy).
  Hence the stage depends on `calibration/camera_info.yaml` (or documented placeholder intrinsics) and
  records `focal_px_used` in `depth_meta.json`.
- `depth_meta.json` minimum: `model`, `scale` ("metres = png * 0.001"), `focal_px_used`, `intrinsics_source`,
  `image_size`, `frame_count`, `created_utc`, `runtime_s`, `gpu`.

### 3.5 Trajectories and maps (every SLAM stage)
- `trajectory_tum.txt`: TUM format, one line per pose, `timestamp_s tx ty tz qx qy qz qw`, timestamp = `t_ns/1e9`
  on the phone clock, pose of the frame named in `trajectory_meta.json` in the SLAM `map` frame.
- `trajectory_meta.json` minimum: `{"frame": "camera"|"body", "method": "...", "mode": "...", "tracking_lost_count": N,
  "scale": "metric"|"arbitrary", "notes": "..."}`. Monocular without IMU is **not** metric; say so.
- RTAB-Map also writes `map.db` (the database: keep it, it is the real map) and `grid.pgm` + `grid.yaml`
  (Nav2 `map_server` format). Keep both; they are different artifacts.

---------------------------------------------------------------------------------------------------------------------

## 4. manifest.json

Formal shape: `manifest.schema.json`. Filled example: `manifest.example.json`. Summary:

| key | meaning |
|---|---|
| `contract_version` | `"1.0"`. Bumped on breaking changes. |
| `session_id`, `label`, `created_utc` | identity |
| `source` | what recorded it: recorder + version, phone model/Android SDK/app version, robot description |
| `time_base` | clock name, unit, first/last timestamp, pointer to `clock_sync.json` |
| `streams` | per stream (`rgb`, `video_raw`, `imu`, `encoders`, `depth`): `present`, path, counts, rates, gaps, verdicts |
| `calibration` | `camera`, `imu_camera`, `wheel`: each `status` = `missing` / `placeholder` / `calibrated` + path |
| `stages` | one record per stage (below) |
| `bundles` | optional named subsets for transfer between machines |

### Stage record
`status` is mandatory: `not_run`, `queued`, `running`, `done`, `failed`, `skipped`, or `placeholder`.
Optional: `machine{hostname,os}`, `started_utc`, `finished_utc`, `tool{name,version,commit}`, `params{}`,
`inputs[]`, `outputs[]`, `log`, `metrics{}`, `error`, plus for placeholders `owner` and `todo[]`.

### Rules for writing it
1. **Write atomically**: write `manifest.json.tmp`, then rename over `manifest.json`.
2. A stage updates **only its own entry** (and `streams.depth` / `calibration` when it creates those). Re-read
   the file right before writing (another machine's stage may have changed it).
3. Set `running` when starting, `done`/`failed` when finishing; on `failed` fill `error` and keep the log.
4. Never delete another stage's entry or outputs.
5. When a session is copied between machines the manifest travels with it and stays the source of truth.

---------------------------------------------------------------------------------------------------------------------

## 5. Stages

`doctor stage` = the name `doctor.py` reports readiness for. "Needs" = what must exist before the stage can run.

| stage | needs | produces | where it runs |
|---|---|---|---|
| `prepare` | raw files (and frames, or `video.h264`+`video.jsonl` to re-extract them) | EuRoC-style files, `sync_report.txt`, manifest `streams` | any Python machine |
| `depth_da3` | RGB frames, intrinsics | `depth/`, `depth_meta.json` | the DA3 machine (CUDA GPU) |
| `rtabmap_offline` | RGB frames + `depth/` (+ encoder/IMU later) | `slam/rtabmap/*` | ROS 2 + RTAB-Map machine (WSL2) |
| `orbslam3` | `mav0/`, `timestamps.txt`, settings yaml | `slam/orbslam3/*` | **PLACEHOLDER** (teammate's machine) |
| `eval` | trajectories from 2+ methods | `eval/*` | future, any Python machine |
| `live_ingest` | phone streaming to the receiver | ROS topics (below) + optional recording | ROS 2 machine |
| `rtabmap_live` | live topics (+ depth later) | live map, saved `map.db` | ROS 2 machine, later |
| `nav2` | `map.db`/`grid`, odometry, localisation | navigation | ROS 2 machine, stretch goal |
| `vlm` | map/images + text query | goal pose | **PLACEHOLDER** |

**Live ROS 2 topics** (published by `navsense_bridge`; the replay node publishes the same names):

| topic | type | content |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | portrait, `bgr8` or `mono8`, stamped with capture time |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | only when a real calibration file is supplied (never invented) |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | `32FC1` metres, replay only (live depth later) |
| `/imu/data_raw` | `sensor_msgs/Imu` | merged gyro+accel, frame `imu`, no orientation |
| `/wheel_ticks` | `sensor_msgs/JointState` | `position = [left, right]` cumulative pulses |
| `/cmd_vel` | `geometry_msgs/Twist` | in. Mapped to the car's `w`/`a`/`d`/stop (bang-bang, forward only) |

### Stage runner interface (the part a hub can rely on)
Every stage is runnable as a command with this shape (names are a proposal; the semantics are the contract):

```
<stage_command> --session <folder> [--params <json file>] [--dry-run]
```
- Exit code `0` = done, non-zero = failed. Print progress to stdout; the runner also tees it to `logs/<stage>.log`.
- On start/finish it updates its manifest entry (section 4 rules).
- `--dry-run` checks inputs and prints what it would do, without writing outputs.
- It must tolerate partial sessions and say clearly which input is missing.

---------------------------------------------------------------------------------------------------------------------

## 6. Moving sessions between machines

- Default: copy/zip the whole folder (shared drive, USB, scp). Check integrity with a `SHA256SUMS` file if the
  transfer is unreliable.
- **`slam_bundle`** (minimum for a machine that only runs ORB-SLAM3): `manifest.json`, `mav0/`, `timestamps.txt`,
  `orbslam3_phone.yaml`, `calibration/`, `sync_report.txt`. Results come back as `slam/orbslam3/` (+ manifest entry).
- The DA3 machine needs `mav0/cam0/data/` and `calibration/` and returns `depth/` + `depth_meta.json`.
- Large raw files (`video.h264`) are optional in transfers: frames can always be regenerated with
  `extract_frames.py` if the raw video is present on the original machine.

## 7. Machine capabilities (`doctor.py`)

Each machine runs `python doctor.py --out capabilities.json`. The report lists every check (`ok`/`warn`/`fail`/
`todo`/`skip`) and a `stages` object: `{"depth_da3": {"ready": true, "blocked_by": []}, ...}` with
`ready: null` = not checked. A hub reads these files to enable or grey out stages per machine and to decide where a
stage can run. `todo` and `skip` on a stage's gating checks mean "not available here".

---------------------------------------------------------------------------------------------------------------------


## 8. ORB-SLAM3 Integration

ORB-SLAM3 is integrated into NavSenseHub for offline RGB-D mapping using recorded phone RGB images and DA3-generated depth maps.

### Machine and Build

- **OS:** EndeavourOS Linux (Arch-based).
- **ORB-SLAM3:** Built locally from source at `~/opt/orbslam-deps/ORB_SLAM3`.
- **Version/commit:** Not recorded.
- **Build:** CMake with OpenCV, Eigen3, Pangolin, Boost Serialization, and required shared libraries.
- **ROS wrapper:** None. ORB-SLAM3 runs directly as a C++ executable.
- **Executable:** `orbslam/build/test_sample1_rgbd`.
- **Doctor:** Implemented in `NavSenseHub/doctor.py`. Checks the runner, executable, settings YAML, ORB vocabulary, and shared libraries. Reports `orbslam3: ready` when requirements are satisfied.

### Supported Modes

| Mode | Status |
|---|---|
| Monocular | Not integrated |
| Monocular-inertial | Not integrated |
| RGB-D | Implemented and tested |

RGB-D mapping can be launched directly:

```bash
./orbslam/build/test_sample1_rgbd \
    "./NavSenseLogger/sessions/20261119_115854"
```

Or through the Python session runner:

```bash
python NavSenseHub/run_orbslam3.py \
    --session "./NavSenseLogger/sessions/20261119_115854"
```

The session can also be selected through the **ORB-SLAM3 Run** button in NavSenseHub.

### Settings and Calibration

- **Settings file:** `orbslam/orbslam3_tecno.yaml`.
- **Camera:** Tecno CLA6, recording at 720 × 1280. Newer recordings use Infinix Note 30 Pro (X678B), 1280 × 720.
- **Intrinsics source:** Phone camera calibration stored in `NavSenseLogger/calibration/[PHONE].yaml`.
- **Depth:** 16-bit PNG depth maps in millimetres, generated using Depth Anything 3.
- **Depth factor:** `RGBD.DepthMapFactor: 1000.0`.
- **IMU noise and Tbc:** Placeholder values in the settings file. Not used by the current RGB-D pipeline.

### Input Loading

The executable accepts a NavSense session directory directly, without an interactive dataset selection menu.

Inputs:

- RGB images: `mav0/cam0/data/<timestamp>.jpg`
- Depth images: `depth/<timestamp>.png`
- Session manifest: `manifest.json`

The loader supports the session's JPEG RGB images and corresponding PNG depth maps. Frame timestamps are obtained from the recorded image filenames rather than generated from the frame index.

### Outputs

Results are saved separately for each session under:

```text
<session>/slam/orbslam3/
├── atlas.osa
├── trajectory_tum.txt
├── keyframe_trajectory_tum.txt
├── trajectory_meta.json
└── orbslam3_run.yaml
```

The trajectory is exported in TUM format:

`timestamp tx ty tz qx qy qz qw`

The metadata specifies:

- `frame`: `camera`
- `method`: `ORB-SLAM3 RGB-D`
- `mode`: `mapping`
- `scale`: `metric`
- Processed frames and tracking statistics

Execution logs are stored in `<session>/logs/orbslam3.log`.

The Python runner also updates the `orbslam3` stage in `manifest.json` with execution status and output paths.

### Viewer

ORB-SLAM3 uses the Pangolin viewer during execution to visualize the camera trajectory and reconstructed map.

- **Start:** Automatically when running the RGB-D executable.
- **Save:** Atlas and trajectory files are saved when execution completes successfully.
- **Stop:** The executable finishes after processing the session. The Hub also provides job cancellation, although child-process termination still needs improvement.

### Known Problems

- Tracking may become unstable during longer recordings, resulting in `RECENTLY_LOST`, `LOST`, and repeated relocalization attempts.
- Some reconstructed maps become distorted before tracking is lost.
- Experimental modifications to ORB-SLAM3's internal localization code caused repeated map resets. The earlier source backups were restored.
- The current RGB-D implementation uses DA3-generated depth rather than physical depth measurements.
- Tracking statistics in `trajectory_meta.json` are not yet fully reliable. In one test, `tracking_lost_count` was reported as zero despite visible tracking failures.
- Monocular and visual-inertial modes have not been integrated into the current session-based pipeline.

**Current status:** RGB-D session execution, Atlas and trajectory export, Doctor readiness checks, and NavSenseHub Run-button integration are implemented. Improving tracking accuracy remains future work.

## 9. VLM / language grounding

**Inputs:**
- `mav0/cam0/data/` — RGB frames (every Nth used; default N=10)
- `semantic/frame_embeddings.npz` — pre-computed per-frame embeddings (OpenCLIP or DINOv2)
- A text query string (passed at runtime, not stored in the session)

**Outputs** (written to `semantic/`):

| file | content |
|------|---------|
| `landmarks.json` | `[{name, pose_map, source_frames, confidence}]` — named places and their map-frame poses |
| `query_result.json` | result of the latest query: matched frame, pose, confidence, top-k alternatives |
| `frame_embeddings.npz` | cached embeddings (keyed by frame timestamp ns) |

**Manifest stage:** `vlm`  
**Stage inputs:** `streams.rgb.present = true`, `semantic/frame_embeddings.npz`  
**Stage outputs:** updates `semantic/landmarks.json`, `semantic/query_result.json`

**Interface contract:** the stage accepts `--session <folder> --query "text"` and exits 0 with
`query_result.json` written, or non-zero on failure. Pose format: same as trajectory entries
(`tx ty tz qx qy qz qw` in the map frame). Confidence is a float in [0, 1].

Open: whether DINOv2 (re-localisation) and OpenCLIP (semantic retrieval) share one stage or split
into `vlm_index` and `vlm_query`; how the goal pose reaches Nav2 (action client vs topic).

---------------------------------------------------------------------------------------------------------------------

## 10. Versioning

- `contract_version` is in every manifest. Additive changes (new optional keys, new stages) keep `1.x`.
  Renames/removals bump the major version and need a migration note here.
- Changelog: **1.0** first version, written to match receiver v0.4, `navsense_bridge` 0.1 and the current app.
