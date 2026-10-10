#!/usr/bin/env python3
"""
NavSense receiver v0.3  (logging + keyboard drive control)
  pip install av opencv-python numpy      (av + cv2 optional: without them it just records)

DRIVE:  w / a / d = go.   EVERY other key (space, s, Enter, Esc, anything) = STOP.
        Keys are read from this terminal (Windows) and from the video window.
        A heartbeat is sent 10x/s from its own thread; the phone stops the car if it goes quiet.

Saves per session: video.h264, video.jsonl, imu.jsonl, events.jsonl (incl. car replies).
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
    def __init__(self, root, mono_off, decode, display, ctrl=None):
        self.dir = Path(root) / time.strftime("%Y%m%d_%H%M%S")
        self.dir.mkdir(parents=True, exist_ok=True)
        self.video = open(self.dir / "video.h264", "wb")
        self.vindex = open(self.dir / "video.jsonl", "w")
        self.imu = open(self.dir / "imu.jsonl", "w")
        self.events = open(self.dir / "events.jsonl", "w", buffering=1)
        self.mono_off = mono_off
        self.ctrl = ctrl
        self.display = display and cv2 is not None
        self.decoder = None
        if decode and av is not None:
            self.decoder = av.CodecContext.create("h264", "r")
            try:
                self.decoder.thread_type = "SLICE"   # frame threading would add frames of delay
            except Exception:
                pass
        self.n_video = self.n_imu = self.n_pkts = self.bytes = 0
        self.dropped = 0
        self.lat = {k: deque(maxlen=300) for k in ("cam_enc", "enc_send", "net", "dec", "e2e", "imu")}
        self.snap = (time.time(), 0, 0, 0, 0)
        self.last_print = 0.0
        self.frames_dir = self.dir / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.frame_index = 0

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
                if h.get("ts_source") != "realtime":
                    print("WARNING: camera timestamps not REALTIME; cam->enc/e2e numbers are unreliable")
            elif typ == "car_reply":
                print("\nCar:", h.get("text", "").strip())
            h["laptop_rx_ms"] = now_ms
            self.events.write(json.dumps(h, separators=(",", ":")) + "\n")

    def decode(self, payload):
        if not self.decoder:
            return []
        try:
            return self.decoder.decode(av.Packet(payload))
        except Exception:
            return []  # e.g. joined mid-GOP; recovers at next keyframe

    def on_video(self, h, payload, now_ms):
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
        frames = self.decode(payload)
        dec_ms = (time.perf_counter() - t) * 1e3
        for f in frames:
            img = f.to_ndarray(format="bgr24")

            # Save each decoded frame as a JPEG.
            self.frame_index += 1
            filename = self.frames_dir / f"frame_{self.frame_index:06d}.jpg"
            cv2.imwrite(str(filename), img, [cv2.IMWRITE_JPEG_QUALITY, 95])

            if self.display:
                cv2.imshow("NavSense", img)
                k = cv2.waitKey(1) & 0xFF
                if k != 255 and self.ctrl:
                    self.ctrl.key(chr(k))

            self.lat["e2e"].append(
                time.time() * 1e3 - (cam - self.mono_off)
            )

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
            f.close()


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
        session = Session(args.output, mono_off, not args.no_decode, not args.no_display)
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
    args = ap.parse_args()
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