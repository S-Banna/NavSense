#!/usr/bin/env python3
"""
NavSense receiver v0.3  (logging + keyboard drive control)
  pip install av opencv-python numpy      (av + cv2 optional: without them it just records)

DRIVE:  w / a / d = go.   EVERY other key (space, s, Enter, Esc, anything) = STOP.
        Keys are read from this terminal (Windows) and from the video window.
        A heartbeat is sent 10x/s from its own thread; the phone stops the car if it goes quiet.

Frames are decoded as they arrive, rotated to portrait, and written straight to JPG.
At session end an ORB-SLAM3 (EuRoC-style) dataset is exported automatically.
Saves per session: mav0/cam0/data/<t_cam_ns>.jpg (decoded frames, named by capture time on the phone's
clock = same clock as imu.jsonl t_ns), video.h264 (raw stream backup), video.jsonl, imu.jsonl,
events.jsonl (incl. car replies).
"""
import argparse
import json
import socket
import struct
import threading
import time
from collections import deque
from pathlib import Path

try:
    import av
except ImportError:
    av = None
try:
    import cv2
except ImportError:
    cv2 = None
try:
    import msvcrt          # Windows console key reads
except ImportError:
    msvcrt = None

SENSORS = ("accelerometer", "gyroscope", "magnetometer")


def recv_exact(sock, n):
    buf = bytearray(n)
    view = memoryview(buf)
    got = 0
    while got < n:
        r = sock.recv_into(view[got:], min(n - got, 1 << 20))
        if r == 0:
            raise ConnectionError("peer disconnected")
        got += r
    return bytes(buf)


def recv_record(sock):
    n = struct.unpack(">I", recv_exact(sock, 4))[0]
    header = json.loads(recv_exact(sock, n))
    plen = int(header.get("payload_len", 0))
    return header, (recv_exact(sock, plen) if plen else b"")


def send_record(sock, obj, payload=b""):
    meta = json.dumps(obj, separators=(",", ":")).encode()
    sock.sendall(struct.pack(">I", len(meta)) + meta + payload)


def calibrate(conn, rounds=16):
    """Returns (rtt, wall_offset_ms, mono_offset_ms) = phone - laptop, from the lowest-RTT round."""
    best = None
    conn.settimeout(2.0)
    for _ in range(rounds):
        t0 = time.time() * 1e3
        send_record(conn, {"type": "sync_req"})
        while True:
            h, _ = recv_record(conn)
            if h.get("type") == "sync_resp":
                break
        t3 = time.time() * 1e3
        w1, w2 = float(h["phone_recv_wall_ms"]), float(h["phone_send_wall_ms"])
        m1, m2 = h["phone_recv_mono_ns"] / 1e6, h["phone_send_mono_ns"] / 1e6
        rtt = (t3 - t0) - (m2 - m1)
        mid = (t0 + t3) / 2
        cand = (rtt, (w1 + w2) / 2 - mid, (m1 + m2) / 2 - mid)
        if best is None or cand[0] < best[0]:
            best = cand
    conn.settimeout(None)
    return best


def med(d):
    return f"{sorted(d)[len(d) // 2]:5.1f}" if d else "  n/a"


def r_body_cam(sensor_orientation, applied_cw):
    """Rotation of the (OpenCV-convention) camera frame of the SAVED image, expressed in the Android
    device/IMU frame (x right, y up, z out of the screen, phone held portrait). Back camera."""
    import numpy as np

    def M(d):
        r = np.deg2rad(d)
        c, s_ = round(float(np.cos(r))), round(float(np.sin(r)))   # exact for multiples of 90
        return np.array([[c, s_], [-s_, c]], dtype=float)
    N = M(sensor_orientation).T @ M(applied_cw)
    xs = np.array([N[0, 0], -N[1, 0], 0.0])
    ys = np.array([N[0, 1], -N[1, 1], 0.0])
    zs = np.array([0.0, 0.0, -1.0])
    return np.stack([xs, ys, zs], axis=1) + 0.0


def export_euroc(sdir, timeshift_ms=0.0, static_seconds=0.0, noise_scale=5.0):
    """Turn a finished session into an ORB-SLAM3-ready EuRoC-style dataset:
         mav0/cam0/data/<ts_ns>.jpg   (already written live)   mav0/cam0/data.csv
         mav0/imu0/data.csv           (accel+gyro merged onto gyro timestamps, EuRoC columns)
         timestamps.txt, orbslam3_phone.yaml, sync_report.txt
    """
    import numpy as np
    sdir = Path(sdir)
    cam_dir = sdir / "mav0" / "cam0" / "data"
    imu_dir = sdir / "mav0" / "imu0"
    out = []

    def say(msg=""):
        out.append(msg)
        print(msg)

    files = sorted([f for f in cam_dir.glob("*") if f.suffix in (".jpg", ".png") and f.stem.isdigit()],
                   key=lambda f: int(f.stem))
    if not files or not (sdir / "imu.jsonl").exists():
        print(f"\nexport: need camera frames in {cam_dir} and imu.jsonl; nothing to export")
        return
    ext = files[0].suffix
    cam_t = np.array([int(f.stem) for f in files], dtype=np.int64)

    at, av_, gt, gv = [], [], [], []
    with open(sdir / "imu.jsonl") as f:
        for line in f:
            r = json.loads(line)
            if r["sensor"] == "accelerometer":
                at.append(r["t_ns"]); av_.append(r["v"])
            elif r["sensor"] == "gyroscope":
                gt.append(r["t_ns"]); gv.append(r["v"])
    if len(at) < 10 or len(gt) < 10:
        print("\nexport: not enough IMU samples")
        return

    def clean(t, v):
        t = np.array(t, dtype=np.int64)
        v = np.array(v, dtype=np.float64)
        bad = int(np.sum(np.diff(t) <= 0))
        o = np.argsort(t, kind="stable")
        t, v = t[o], v[o]
        keep = np.concatenate(([True], np.diff(t) > 0))
        return t[keep], v[keep], bad
    at, av_, bad_a = clean(at, av_)
    gt, gv, bad_g = clean(gt, gv)

    # Merge: ORB-SLAM3 wants one 6-axis sample per timestamp. Android reports accel and gyro
    # separately with their own timestamps, so interpolate accel onto the gyro timestamps.
    sel = (gt >= at[0]) & (gt <= at[-1])
    mt, mg = gt[sel], gv[sel]
    ma = np.stack([np.interp((mt - at[0]).astype(np.float64), (at - at[0]).astype(np.float64), av_[:, k])
                   for k in range(3)], axis=1)
    shift_ns = int(round(timeshift_ms * 1e6))
    mt = mt - shift_ns      # Kalibr: t_imu = t_cam + shift  ->  express IMU on the camera clock

    imu_dir.mkdir(parents=True, exist_ok=True)
    with open(imu_dir / "data.csv", "w") as f:
        f.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
        for t, g, a in zip(mt, mg, ma):
            f.write(f"{t},{g[0]:.9g},{g[1]:.9g},{g[2]:.9g},{a[0]:.9g},{a[1]:.9g},{a[2]:.9g}\n")

    # Only frames fully covered by IMU (the stock loader needs an IMU sample before the first frame).
    keep = (cam_t >= mt[0]) & (cam_t <= mt[-1])
    ct = cam_t[keep]
    with open(sdir / "mav0" / "cam0" / "data.csv", "w") as f:
        f.write("#timestamp [ns],filename\n")
        for t in ct:
            f.write(f"{t},{t}{ext}\n")
    with open(sdir / "timestamps.txt", "w") as f:
        f.write("\n".join(str(t) for t in ct) + "\n")

    # ---- diagnostics
    dts = np.diff(mt) / 1e9
    imu_hz = 1.0 / float(np.median(dts))
    cdt = np.diff(ct) / 1e9
    fps = 1.0 / float(np.median(cdt)) if len(cdt) else 0.0
    per = np.diff(np.searchsorted(mt, ct)) if len(ct) > 1 else np.array([0])
    gnorm = float(np.linalg.norm(ma, axis=1).mean())
    say("\n=== ORB-SLAM3 sync report ===")
    say(f"frames kept {len(ct)}/{len(cam_t)} | span {(ct[-1]-ct[0])/1e9:.1f} s | median fps {fps:.1f}")
    say(f"IMU merged samples {len(mt)} | rate {imu_hz:.1f} Hz | raw accel {len(at)} / gyro {len(gt)} samples")
    say(f"IMU samples between consecutive frames: min {per.min()} / median {int(np.median(per))} / max {per.max()}"
        + ("   <-- zero-IMU intervals!" if per.min() == 0 else ""))
    mg_cam = float(cdt.max()) if len(cdt) else 0.0
    say(f"largest gap between frames: {mg_cam*1000:.0f} ms"
        + ("   <-- >= ~1 s makes ORB-SLAM3 call it a timestamp jump and reset the map" if mg_cam >= 0.9 else
           ("   <-- big gap, expect tracking trouble" if mg_cam > 0.25 else "")))
    say(f"frames with gap > 100 ms: {int((cdt > 0.1).sum())}")
    say(f"largest IMU gap: {dts.max()*1000:.0f} ms | IMU gaps > 20 ms: {int((dts > 0.02).sum())}"
        f" | out-of-order/duplicate stamps removed: accel {bad_a}, gyro {bad_g}")
    say(f"mean |accel| {gnorm:.2f} m/s^2 (expect ~9.8)"
        + ("" if 9.0 < gnorm < 10.6 else "   <-- units/scale look wrong"))
    if timeshift_ms:
        say(f"applied camera-IMU timeshift {timeshift_ms:+.2f} ms (IMU stamps moved by {-timeshift_ms:+.2f} ms)")
    else:
        say("no camera-IMU timeshift applied (see --timeshift-ms; calibrate with Kalibr)")

    # ---- noise + config
    dt = float(np.median(dts))
    if static_seconds > 0:
        w = (mt - mt[0]) <= static_seconds * 1e9
        ng = float(np.mean(np.std(mg[w], axis=0)) * np.sqrt(dt))
        na = float(np.mean(np.std(ma[w], axis=0)) * np.sqrt(dt))
        src = f"measured from first {static_seconds:g} s (phone must have been still)"
    else:
        ng, na, src = 1.7e-4, 2.0e-3, "EuRoC defaults"
    ng, na = ng * noise_scale, na * noise_scale
    gw, aw = 1.9393e-5 * noise_scale, 3.0e-3 * noise_scale
    say(f"IMU noise: {src}, inflated x{noise_scale:g}: gyro {ng:.2e}, acc {na:.2e}, walks {gw:.2e}/{aw:.2e}")

    meta = {}
    if (sdir / "session_meta.json").exists():
        meta = json.load(open(sdir / "session_meta.json"))
    so, rot = int(meta.get("sensor_orientation", 90)), int(meta.get("rotation_cw_deg", 90))
    R = r_body_cam(so, rot)
    w_px = h_px = 0
    if cv2 is not None:
        im = cv2.imread(str(files[0]))
        if im is not None:
            h_px, w_px = im.shape[:2]
    fx = fy = round(1.05 * min(w_px, h_px), 1) if w_px else 700.0

    y = ["%YAML:1.0", 'File.version: "1.0"', "",
         "# GENERATED by navsense_receiver. Items marked REPLACE need real calibration.", "",
         'Camera.type: "PinHole"',
         f"Camera1.fx: {fx}   # REPLACE: calibrate with a checkerboard at THIS resolution/orientation",
         f"Camera1.fy: {fy}   # REPLACE",
         f"Camera1.cx: {w_px / 2:.1f}   # REPLACE",
         f"Camera1.cy: {h_px / 2:.1f}   # REPLACE",
         "Camera1.k1: 0.0", "Camera1.k2: 0.0", "Camera1.p1: 0.0", "Camera1.p2: 0.0",
         f"Camera.fps: {int(round(fps))}", "Camera.RGB: 0",
         f"Camera.width: {w_px}", f"Camera.height: {h_px}", "",
         "# camera -> body(IMU). Rotation derived from Android's device frame; TRANSLATION IS 0: REPLACE (Kalibr/measure)",
         "IMU.T_b_c1: !!opencv-matrix", "   rows: 4", "   cols: 4", "   dt: f", "   data: [" +
         ",\n          ".join(", ".join(f"{R[i, j]:.1f}" for j in range(3)) + ", 0.0" for i in range(3)) +
         ",\n          0.0, 0.0, 0.0, 1.0]", "",
         f"IMU.NoiseGyro: {ng:.4e}", f"IMU.NoiseAcc: {na:.4e}",
         f"IMU.GyroWalk: {gw:.4e}", f"IMU.AccWalk: {aw:.4e}", f"IMU.Frequency: {imu_hz:.1f}", "",
         "ORBextractor.nFeatures: 1200", "ORBextractor.scaleFactor: 1.2", "ORBextractor.nLevels: 8",
         "ORBextractor.iniThFAST: 20", "ORBextractor.minThFAST: 7", "",
         "Viewer.KeyFrameSize: 0.05", "Viewer.KeyFrameLineWidth: 1.0", "Viewer.GraphLineWidth: 0.9",
         "Viewer.PointSize: 2.0", "Viewer.CameraSize: 0.08", "Viewer.CameraLineWidth: 3.0",
         "Viewer.ViewpointX: 0.0", "Viewer.ViewpointY: -0.7", "Viewer.ViewpointZ: -1.8",
         "Viewer.ViewpointF: 500.0", "Viewer.imageViewScale: 1.0"]
    (sdir / "orbslam3_phone.yaml").write_text("\n".join(y) + "\n")
    say(f"config: orbslam3_phone.yaml  ({w_px}x{h_px}, fps {int(round(fps))}, IMU {imu_hz:.0f} Hz, "
        f"sensor orientation {so}, image rotation {rot})")
    say("Intrinsics, distortion and the camera-IMU translation are PLACEHOLDERS; calibrate them.")
    (sdir / "sync_report.txt").write_text("\n".join(out) + "\n")
    print(f"export done -> {sdir}")


class Control:
    """Holds the current drive command and sends it to the phone from its OWN thread.
    Only w/a/d move the car; any other key is a stop. Independent of video, so a stop
    key goes out even if the video stalls."""

    def __init__(self, conn):
        self.conn = conn
        self.cmd = "s"
        self.dirty = True
        self.sent = 0                       # heartbeats sent (shown as hb= in the status line)
        self.stop_evt = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def key(self, ch):
        ch = ch.lower()
        new = ch if ch in ("w", "a", "d") else "s"     # every other key = kill
        if new != self.cmd:
            self.cmd = new
            self.dirty = True
            print(f"\n[cmd] {new}", flush=True)

    def _loop(self):
        last_send = 0.0
        while not self.stop_evt.is_set():
            if msvcrt:
                while msvcrt.kbhit():
                    self.key(msvcrt.getwch())
            now = time.time()
            if self.dirty or now - last_send >= 0.1:
                self.dirty = False
                last_send = now
                try:
                    send_record(self.conn, {"type": "cmd", "c": self.cmd})
                    self.sent += 1
                except OSError:
                    return
            time.sleep(0.01)

    def shutdown(self):
        self.stop_evt.set()
        self.thread.join(0.3)
        self.cmd = "s"
        for _ in range(3):                              # best-effort explicit stop
            try:
                send_record(self.conn, {"type": "cmd", "c": "s"})
            except OSError:
                break


class Session:
    def __init__(self, root, mono_off, decode, display, ctrl=None, frames_fmt="jpg", jpg_q=95,
                 rotate="auto", save_h264=True, export_opts=None):
        self.dir = Path(root) / time.strftime("%Y%m%d_%H%M%S")
        self.dir.mkdir(parents=True, exist_ok=True)
        self.video = open(self.dir / "video.h264", "wb") if save_h264 else None
        self.vindex = open(self.dir / "video.jsonl", "w")
        self.imu = open(self.dir / "imu.jsonl", "w")
        self.events = open(self.dir / "events.jsonl", "w", buffering=1)
        self.mono_off = mono_off
        self.ctrl = ctrl
        self.frames_fmt, self.jpg_q = frames_fmt, jpg_q
        self.rotate = rotate                 # "auto" = upright portrait via the camera's sensor orientation
        self.sensor_orientation = 90         # used until the phone reports it (typical for back cameras)
        self.export_opts = export_opts
        self.camera_dir = self.dir / "mav0" / "cam0" / "data"   # EuRoC layout: file name = capture time (ns)
        self.display = display and cv2 is not None
        self.decoder = None
        if decode and av is not None:
            self.decoder = av.CodecContext.create("h264", "r")
            try:
                self.decoder.thread_type = "SLICE"   # frame threading would add frames of delay
            except Exception:
                pass
        self.save_frames = frames_fmt != "none" and cv2 is not None and self.decoder is not None
        if frames_fmt != "none" and not self.save_frames:
            print("WARNING: saving images needs PyAV and OpenCV installed; not saving frames")
        if self.save_frames:
            self.camera_dir.mkdir(parents=True, exist_ok=True)
        self.n_video = self.n_imu = self.n_pkts = self.bytes = 0
        self.dropped = 0
        self.lat = {k: deque(maxlen=300) for k in ("cam_enc", "enc_send", "net", "dec", "e2e", "imu")}
        self.snap = (time.time(), 0, 0, 0, 0)
        self.last_print = 0.0

    def handle(self, h, payload):
        now_ms = time.time() * 1e3
        typ = h.get("type")
        self.bytes += len(payload)
        if typ == "video":
            self.on_video(h, payload, now_ms)
        elif typ == "imu_batch":
            self.on_imu(h, now_ms)
        else:
            if typ == "video_info":
                print("\nVideo:", h)
                if h.get("sensor_orientation") is not None:
                    self.sensor_orientation = int(h["sensor_orientation"])
                if h.get("ts_source") != "realtime":
                    print("WARNING: camera timestamps not REALTIME; cam->enc/e2e numbers are unreliable")
            elif typ == "car_reply":
                print("\nCar:", h.get("text", "").strip())
            h["laptop_rx_ms"] = now_ms
            self.events.write(json.dumps(h, separators=(",", ":")) + "\n")

    def applied_rotation(self):
        """Clockwise degrees applied to the saved images relative to the landscape sensor image."""
        return (self.sensor_orientation if self.rotate == "auto" else int(self.rotate)) % 360

    def orient(self, img):
        """Rotate the landscape sensor image to upright portrait (or a forced angle)."""
        deg = self.applied_rotation()
        code = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
                270: cv2.ROTATE_90_COUNTERCLOCKWISE}.get(deg % 360)
        return cv2.rotate(img, code) if code is not None else img

    def decode(self, payload, pts=None):
        if not self.decoder:
            return []
        try:
            pkt = av.Packet(payload)
            if pts is not None:
                pkt.pts = pts          # capture time rides along and comes back on the decoded frame
            return self.decoder.decode(pkt)
        except Exception:
            return []  # e.g. joined mid-GOP; recovers at next keyframe

    def on_video(self, h, payload, now_ms):
        if self.video:
            self.video.write(payload)
        if h.get("config"):
            self.decode(payload)
            return
        self.n_video += 1
        self.dropped = h.get("dropped", 0)
        cam, enc, snd = h["t_cam_ns"] / 1e6, h["t_enc_ns"] / 1e6, h["t_send_ns"] / 1e6
        self.lat["cam_enc"].append(enc - cam)
        self.lat["enc_send"].append(snd - enc)
        self.lat["net"].append(now_ms - (snd - self.mono_off))
        h["laptop_rx_ms"] = now_ms
        self.vindex.write(json.dumps(h, separators=(",", ":")) + "\n")
        t = time.perf_counter()
        frames = self.decode(payload, int(h["t_cam_ns"]))
        dec_ms = (time.perf_counter() - t) * 1e3
        for f in frames:
            ts = f.pts if f.pts is not None else int(h["t_cam_ns"])   # capture time, ns, phone clock
            self.lat["dec"].append(dec_ms)
            img = f.to_ndarray(format="bgr24") if (self.display or self.save_frames) else None
            if img is not None:
                img = self.orient(img)
            if self.display:
                cv2.imshow("NavSense", img)
                k = cv2.waitKey(1) & 0xFF
                if k != 255 and self.ctrl:
                    self.ctrl.key(chr(k))               # any key here too; non-wad = stop
            self.lat["e2e"].append(time.time() * 1e3 - (ts / 1e6 - self.mono_off))
            if self.save_frames:
                path = str(self.camera_dir / f"{ts}.{self.frames_fmt}")
                if self.frames_fmt == "jpg":
                    cv2.imwrite(path, img, [cv2.IMWRITE_JPEG_QUALITY, self.jpg_q])
                else:
                    cv2.imwrite(path, img)

    def on_imu(self, h, now_ms):
        s = h["samples"]
        self.imu.write("".join(
            json.dumps({"sensor": SENSORS[k], "t_ns": t, "v": [x, y, z]}, separators=(",", ":")) + "\n"
            for k, t, x, y, z in s))
        self.n_imu += len(s)
        self.n_pkts += 1
        self.lat["imu"].append(now_ms - (h["t_batch_ns"] / 1e6 - self.mono_off))

    def maybe_print(self):
        now = time.time()
        if now - self.last_print < 1:
            return
        self.last_print = now
        t0, v0, i0, p0, b0 = self.snap
        dt = max(now - t0, 1e-6)
        self.snap = (now, self.n_video, self.n_imu, self.n_pkts, self.bytes)
        L = self.lat
        print(
            f"\rVID {(self.n_video - v0) / dt:4.1f}fps {(self.bytes - b0) * 8 / dt / 1e6:5.1f}Mbps | "
            f"IMU {(self.n_imu - i0) / dt:4.0f}/s ({(self.n_pkts - p0) / dt:3.0f} pkt/s) | "
            f"ms: cam>enc {med(L['cam_enc'])} enc>send {med(L['enc_send'])} net {med(L['net'])} "
            f"dec {med(L['dec'])} E2E {med(L['e2e'])} imu-net {med(L['imu'])} | drop {self.dropped} | hb {self.ctrl.sent if self.ctrl else 0}",
            end="", flush=True)

    def close(self):
        for f in (self.video, self.vindex, self.imu, self.events):
            if f:
                f.close()
        try:
            with open(self.dir / "session_meta.json", "w") as f:
                json.dump({"sensor_orientation": self.sensor_orientation,
                           "rotation_cw_deg": self.applied_rotation(),
                           "frames_fmt": self.frames_fmt}, f)
            if self.save_frames and self.export_opts is not None:
                export_euroc(self.dir, **self.export_opts)
        except Exception as e:
            print(f"\nEuRoC export failed: {e}  (re-run with --export {self.dir})")


def handle_client(conn, addr, args):
    print(f"\nClient connected: {addr}")
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    session = None
    ctrl = None
    try:
        hello, _ = recv_record(conn)
        print("Phone hello:", hello)
        rtt, wall_off, mono_off = calibrate(conn)
        print(f"Clock sync: best RTT {rtt:.1f} ms | wall offset {wall_off:+.1f} ms | mono offset {mono_off:+.1f} ms")
        session = Session(args.output, mono_off, not args.no_decode, not args.no_display,
                          frames_fmt=args.frames, jpg_q=args.jpg_quality,
                          rotate=args.rotate, save_h264=not args.no_h264,
                          export_opts=args.export_opts)
        if av is None:
            print("(PyAV not installed: recording only, no decode/latency for display)")
        send_record(conn, {"type": "start"})   # phone begins camera + IMU only now
        ctrl = Control(conn)
        session.ctrl = ctrl
        print("Drive: w/a/d = go, ANY other key = stop (keep this terminal or the video window focused)")
        while True:
            h, payload = recv_record(conn)
            session.handle(h, payload)
            session.maybe_print()
    except Exception as e:
        print(f"\nClient ended: {e}")
    finally:
        if ctrl:
            ctrl.shutdown()                     # always try to leave the car stopped
        conn.close()
        if session:
            session.close()
            print(f"Session saved to {session.dir}")
        if cv2 is not None:
            cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--output", default="sessions")
    ap.add_argument("--no-display", action="store_true")
    ap.add_argument("--no-decode", action="store_true")
    ap.add_argument("--frames", choices=["jpg", "png", "none"], default="png",
                    help="save decoded frames as images (default jpg; png is lossless but slower/larger)")
    ap.add_argument("--jpg-quality", type=int, default=95)
    ap.add_argument("--rotate", choices=["auto", "0", "90", "180", "270"], default="auto",
                    help="auto = upright portrait from the camera's sensor orientation; or force degrees clockwise")
    ap.add_argument("--no-h264", action="store_true", help="don't keep the raw video.h264 backup")
    ap.add_argument("--no-export", action="store_true", help="skip the automatic ORB-SLAM3/EuRoC export at session end")
    ap.add_argument("--export", metavar="SESSION_DIR", help="re-run the export on an existing session folder and exit")
    ap.add_argument("--timeshift-ms", type=float, default=0.0,
                    help="Kalibr 'timeshift cam0 to imu0' in ms (t_imu = t_cam + shift); applied to IMU stamps")
    ap.add_argument("--static-seconds", type=float, default=0.0,
                    help="phone held still for the first N seconds: use them to estimate IMU noise")
    ap.add_argument("--noise-scale", type=float, default=5.0, help="inflation factor on IMU noise values")
    args = ap.parse_args()
    if args.export:
        export_euroc(args.export, args.timeshift_ms, args.static_seconds, args.noise_scale)
        return
    args.export_opts = None if args.no_export else dict(
        timeshift_ms=args.timeshift_ms, static_seconds=args.static_seconds, noise_scale=args.noise_scale)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((args.host, args.port))
        server.listen(1)
        print(f"NavSense receiver listening on {args.host}:{args.port}")
        print(f"Output: {Path(args.output).resolve()}")
        while True:
            conn, addr = server.accept()
            handle_client(conn, addr, args)


if __name__ == "__main__":
    main()