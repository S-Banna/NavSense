#!/usr/bin/env python3
"""Re-create the timestamped frames of a recorded session from video.h264 + video.jsonl (no phone/network needed).

  python extract_frames.py sessions/20261116_101500 --fmt png
Output: <session>/mav0/cam0/data/<capture_time_ns>.<fmt>  (same naming as the live receiver, same phone clock as imu.jsonl)
Needs sessions recorded with receiver v0.4+ (video.jsonl lists every packet with file_offset).
"""
import argparse
import json
import sys
from pathlib import Path

import av
import cv2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--fmt", choices=["jpg", "png"], default="jpg")
    ap.add_argument("--quality", type=int, default=95)
    ap.add_argument("--rotate", choices=["auto", "0", "90", "180", "270"], default="0")
    ap.add_argument("--out", default=None, help="output folder (default <session>/mav0/cam0/data)")
    a = ap.parse_args()
    sd = Path(a.session)
    rows = [json.loads(l) for l in open(sd / "video.jsonl")]
    if not rows or "file_offset" not in rows[0]:
        sys.exit("video.jsonl has no file_offset: this session was recorded by an older receiver; cannot re-decode.")
    if not (sd / "video.h264").exists():
        sys.exit("video.h264 missing (session recorded with --no-h264).")
    deg = 0 if a.rotate == "auto" else int(a.rotate)
    code = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}.get(deg % 360)
    out = Path(a.out) if a.out else sd / "mav0" / "cam0" / "data"
    out.mkdir(parents=True, exist_ok=True)
    blob = (sd / "video.h264").read_bytes()
    dec = av.CodecContext.create("h264", "r")
    try:
        dec.thread_type = "SLICE"
    except Exception:
        pass
    n = 0
    for r in rows:
        pkt = av.Packet(blob[r["file_offset"]: r["file_offset"] + r["payload_len"]])
        if not r.get("config"):
            pkt.pts = int(r["t_cam_ns"])
        try:
            frames = dec.decode(pkt)
        except Exception:
            continue
        for f in frames:
            img = f.to_ndarray(format="bgr24")
            if code is not None:
                img = cv2.rotate(img, code)
            path = str(out / f"{f.pts if f.pts is not None else int(r['t_cam_ns'])}.{a.fmt}")
            if a.fmt == "jpg":
                cv2.imwrite(path, img, [cv2.IMWRITE_JPEG_QUALITY, a.quality])
            else:
                cv2.imwrite(path, img)
            n += 1
            if n % 200 == 0:
                print(f"  {n} frames...", flush=True)
    print(f"wrote {n} frames to {out}")


if __name__ == "__main__":
    main()