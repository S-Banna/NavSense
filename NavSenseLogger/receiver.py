#!/usr/bin/env python3
import argparse
import json
import os
import socket
import struct
import threading
import time
from pathlib import Path
from collections import deque

BUF = 1024 * 1024

def recv_exact(sock, n):
    out = bytearray()
    while len(out) < n:
        chunk = sock.recv(min(BUF, n - len(out)))
        if not chunk:
            raise ConnectionError("peer disconnected")
        out.extend(chunk)
    return bytes(out)

def recv_record(sock):
    raw = recv_exact(sock, 4)
    header_len = struct.unpack(">I", raw)[0]
    header = json.loads(recv_exact(sock, header_len).decode("utf-8"))
    payload_len = int(header.get("payload_len", 0))
    payload = recv_exact(sock, payload_len) if payload_len else b""
    return header, payload

def send_record(sock, obj, payload=b""):
    meta = json.dumps(obj, separators=(",", ":")).encode("utf-8")
    sock.sendall(struct.pack(">I", len(meta)) + meta + payload)

def estimate_clock_offset(conn, rounds=8):
    # Estimate: phone_time - laptop_time.
    # For each request, t0 is laptop send time and t1/t2 are phone receive/send.
    # Midpoint approximation: laptop_mid=(t0+t3)/2.
    samples = []
    conn.settimeout(2.0)
    for _ in range(rounds):
        t0 = time.time() * 1000.0
        send_record(conn, {"type": "sync_req", "t_laptop_send_ms": t0})
        while True:
            h, payload = recv_record(conn)
            if h.get("type") == "sync_resp":
                t3 = time.time() * 1000.0
                t1 = float(h["phone_recv_wall_ms"])
                t2 = float(h.get("phone_send_wall_ms", t1))
                midpoint = (t0 + t3) / 2.0
                samples.append(((t1 + t2) / 2.0) - midpoint)
                break
            # Preserve unexpected records by returning them to the caller is complicated
            # for a stream, so sync should happen before normal logging begins.
    conn.settimeout(None)
    if not samples:
        return 0.0
    samples.sort()
    return samples[len(samples)//2]

class Session:
    def __init__(self, root, clock_offset_ms=0.0):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.dir = Path(root) / stamp
        self.dir.mkdir(parents=True, exist_ok=True)
        self.imu_file = open(self.dir / "imu.jsonl", "w", buffering=1)
        self.events_file = open(self.dir / "events.jsonl", "w", buffering=1)
        self.camera_dir = self.dir / "camera"
        self.camera_dir.mkdir(exist_ok=True)
        self.clock_offset_ms = clock_offset_ms
        self.lock = threading.Lock()
        self.counts = {"camera": 0, "imu": 0, "encoder": 0}
        self.bytes = 0
        self.latencies = deque(maxlen=300)
        self.first_wall = None
        self.last_print = 0

    def handle(self, header, payload):
        typ = header.get("type", "unknown")
        now_ms = time.time() * 1000.0
        self.bytes += len(payload)
        self.counts[typ] = self.counts.get(typ, 0) + 1

        phone_wall = header.get("t_wall_ms")
        # Raw clock-difference latency is displayed only as a diagnostic.
        # A future sync reply can turn this into a calibrated one-way latency.
        raw_latency = None
        if phone_wall is not None:
            # Phone clock is estimated as: phone_time = laptop_time + offset.
            # Therefore estimated one-way transport latency is
            # laptop_receive - (phone_timestamp - offset).
            raw_latency = now_ms - (float(phone_wall) - self.clock_offset_ms)
            self.latencies.append(raw_latency)

        event = dict(header)
        event["laptop_receive_wall_ms"] = now_ms
        if raw_latency is not None:
            event["raw_clock_difference_ms"] = raw_latency

        if typ == "camera":
            idx = self.counts["camera"]
            path = self.camera_dir / f"{idx:07d}_{header.get('t_wall_ms', 0)}.jpg"
            path.write_bytes(payload)
            event["file"] = str(path.relative_to(self.dir))
            try:
                import cv2
                import numpy as np
                frame = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    cv2.imshow("NavSense camera", frame)
                    cv2.waitKey(1)
            except ImportError:
                pass
        elif typ == "imu":
            self.imu_file.write(json.dumps(event, separators=(",", ":")) + "\n")
        else:
            self.events_file.write(json.dumps(event, separators=(",", ":")) + "\n")

    def stats(self):
        now = time.time()
        elapsed = max(now - getattr(self, "_started", now), 1e-6)
        cam_rate = self.counts["camera"] / elapsed
        imu_rate = self.counts["imu"] / elapsed
        mbps = self.bytes * 8 / elapsed / 1e6
        med = sorted(self.latencies)
        median = med[len(med)//2] if med else None
        return cam_rate, imu_rate, mbps, median

    def close(self):
        self.imu_file.close()
        self.events_file.close()

def handle_client(conn, addr, root):
    print(f"\nClient connected: {addr}")
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    try:
        # Phone sends hello first; consume it, then perform clock calibration before data logging.
        hello, _ = recv_record(conn)
        print("Phone hello:", hello)
        offset = estimate_clock_offset(conn)
        print(f"Estimated phone clock offset: {offset:+.2f} ms (phone - laptop)")
        session = Session(root, clock_offset_ms=offset)
        session._started = time.time()
        while True:
            header, payload = recv_record(conn)
            session.handle(header, payload)
            now = time.time()
            if now - session.last_print >= 1:
                session.last_print = now
                cam, imu, mbps, med = session.stats()
                lat = f"{med:.1f} ms" if med is not None else "n/a"
                print(
                    f"\rCAM {cam:5.1f}/s | IMU {imu:6.1f}/s | "
                    f"RX {mbps:6.2f} Mbps | latency {lat} | "
                    f"frames {session.counts['camera']} | samples {session.counts['imu']}",
                    end="",
                    flush=True,
                )
    except Exception as e:
        print(f"\nClient ended: {e}")
    finally:
        conn.close()
        if 'session' in locals():
            session.close()
            print(f"Session saved to {session.dir}")
        try:
            import cv2
            cv2.destroyAllWindows()
        except ImportError:
            pass

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--output", default="sessions")
    args = ap.parse_args()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((args.host, args.port))
        server.listen(1)
        print(f"NavSense receiver listening on {args.host}:{args.port}")
        print(f"Output: {Path(args.output).resolve()}")
        while True:
            conn, addr = server.accept()
            # Prototype 0: one phone at a time.
            handle_client(conn, addr, args.output)

if __name__ == "__main__":
    main()
