"""
stream_quality.py — objective frame-quality report, no panel needed
-------------------------------------------------------------------
Captures the stream for N seconds and scores it:

  transport:  fps, corrupt/truncated %, bytes/frame, throughput used
  image:      sharpness (focus), brightness, contrast
  evidence:   saves a few sample frames as JPGs so you can look later

Works on either source:
    python3 stream_quality.py                          # via S3 (auto port)
    python3 stream_quality.py /dev/cu.usbserial-1140 460800   # camera direct
    python3 stream_quality.py COM5 460800 20           # Windows, 20 seconds

Use it to tune camera firmware: run once, change ONE setting (jpeg_quality,
frame_size, baud), reflash, run again, compare the two reports.

Only pyserial is REQUIRED (pip install pyserial). If opencv-python and numpy
are also installed you additionally get image metrics and sample frames;
without them the transport report still works.

What good looks like (QVGA via S3 at 460800):
  fps 5-10 | corrupt < 5% | sharpness > 100 on a textured scene
Sharpness is scene-dependent: always compare on the SAME scene, ideally a
printed page or checkerboard at a fixed distance.
"""

import os
import statistics
import struct
import sys
import time

import serial
from serial.tools import list_ports

# OpenCV/numpy are OPTIONAL. Without them you still get the full transport
# report; only the image metrics and sample frames need decoding.
try:
    import cv2
    import numpy as np
    HAVE_CV = True
except ImportError:
    HAVE_CV = False

PORT = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] else None
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 921600
SECONDS = int(sys.argv[3]) if len(sys.argv) > 3 else 10

SAMPLE_DIR = os.path.expanduser("~/Downloads/pipe_cam_quality_samples")
MAX_FRAME = 300_000
CAM_IDS = {b"\x00": "L", b"\x01": "R", b"L": "L", b"R": "R"}


def find_port():
    if PORT:                       # "" means "auto-detect but I passed baud"
        return PORT
    cands = [p.device for p in list_ports.comports()
             if any(k in p.device.lower()
                    for k in ("usbmodem", "usbserial", "ttyacm", "ttyusb"))]
    for p in list_ports.comports():
        print(f"  seen: {p.device}  ({p.description})")
    return cands[0] if cands else None


def parse_frames(data, gaps=None):
    """Extract JPEG payloads from a capture. Handles BOTH formats:
    tagged (via S3: AA55 id len payload) and untagged (camera direct:
    AA55 len payload). Returns (frames, corrupt_count) where frames is
    a list of (cam_id, jpeg_bytes).

    If `gaps` is a list, byte ranges that were NOT part of any frame get
    appended to it — that's how we find out what the unparsed traffic is."""
    frames, corrupt = [], 0
    i = 0
    n = len(data)
    last_end = 0
    while True:
        i = data.find(b"\xAA\x55", i)
        if i < 0 or i + 6 > n:
            break

        # Try tagged first: id byte then length
        parsed = False
        raw_id = data[i + 2:i + 3]

        # Telemetry (id 2) is legitimate traffic, not junk — consume it so it
        # doesn't inflate the "unaccounted" figure.
        if raw_id == b"\x02" and i + 7 <= n:
            (tlen,) = struct.unpack("<I", data[i + 3:i + 7])
            if 0 < tlen < 1000 and i + 7 + tlen <= n:
                if gaps is not None and i > last_end:
                    gaps.append(data[last_end:i])
                i += 7 + tlen
                last_end = i
                continue

        if raw_id in CAM_IDS and i + 7 <= n:
            (length,) = struct.unpack("<I", data[i + 3:i + 7])
            if 0 < length <= MAX_FRAME:
                payload = data[i + 7:i + 7 + length]
                if len(payload) == length:
                    if payload[:2] == b"\xFF\xD8" and b"\xFF\xD9" in payload:
                        frames.append((CAM_IDS[raw_id], payload))
                    else:
                        corrupt += 1
                    if gaps is not None and i > last_end:
                        gaps.append(data[last_end:i])
                    i += 7 + length
                    last_end = i
                    parsed = True

        # Untagged: length immediately after magic
        if not parsed:
            (length,) = struct.unpack("<I", data[i + 2:i + 6])
            if 0 < length <= MAX_FRAME:
                payload = data[i + 6:i + 6 + length]
                if len(payload) == length:
                    if payload[:2] == b"\xFF\xD8" and b"\xFF\xD9" in payload:
                        frames.append(("?", payload))
                    else:
                        corrupt += 1
                    i += 6 + length
                    parsed = True

        if not parsed:
            i += 2
    if gaps is not None and last_end < n:
        gaps.append(data[last_end:])
    return frames, corrupt


def image_metrics(jpeg):
    """Decode and score one frame. Returns None if it won't decode."""
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return None
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Truncated JPEGs often decode with a gray bottom: measure how much of
    # the image is one flat value row-by-row at the bottom.
    row_std = gray.std(axis=1)
    flat_rows = 0
    for v in row_std[::-1]:
        if v < 2.0:
            flat_rows += 1
        else:
            break
    return {
        "shape": img.shape,
        "sharpness": float(cv2.Laplacian(gray, cv2.CV_64F).var()),
        "brightness": float(gray.mean()),
        "contrast": float(gray.std()),
        "flat_bottom_pct": 100.0 * flat_rows / gray.shape[0],
        "img": img,
    }


def main():
    port = find_port()
    if not port:
        print("No serial port found.")
        return
    print(f"Capturing {SECONDS}s from {port} at {BAUD} baud...")
    ser = serial.Serial(port, BAUD, timeout=0.1)
    data = bytearray()
    t0 = time.time()
    while time.time() - t0 < SECONDS:
        data.extend(ser.read(8192))
    ser.close()
    data = bytes(data)

    gaps = []
    frames, corrupt = parse_frames(data, gaps)
    total = len(frames) + corrupt

    print()
    print("================ TRANSPORT ================")
    print(f"bytes captured : {len(data)}  ({len(data)/SECONDS:.0f} B/s, "
          f"{100*len(data)*10/(SECONDS*BAUD):.0f}% of {BAUD} baud)")
    if total == 0:
        print("NO frames found. Wrong baud for this source? (S3 via USB: any "
              "baud OK; camera direct: must match its Serial.begin)")
        return
    print(f"frames         : {len(frames)} good, {corrupt} corrupt "
          f"({100*corrupt/total:.1f}% corrupt)")
    print(f"fps (good)     : {len(frames)/SECONDS:.1f}")
    sizes = [len(j) for _, j in frames]
    if sizes:
        print(f"frame size     : avg {statistics.mean(sizes)/1000:.1f} KB   "
              f"min {min(sizes)/1000:.1f}   max {max(sizes)/1000:.1f}")
        by_cam = {}
        for cam, _ in frames:
            by_cam[cam] = by_cam.get(cam, 0) + 1
        print(f"per camera     : {by_cam}")

    # Bytes that never became a frame. A little is normal (partial frame at
    # the start/end of the capture window). A LOT means the stream is badly
    # desynced — the parser is throwing away most of what arrives.
    accounted = sum(sizes) + (len(frames) + corrupt) * 7
    unaccounted = max(0, len(data) - accounted)
    print(f"unaccounted    : {unaccounted/1000:.1f} KB "
          f"({100*unaccounted/len(data):.0f}% of traffic never parsed as a frame)")

    if len(frames) < 20:
        print(f"\n!! only {len(frames)} frames in {SECONDS}s — too few to draw "
              f"conclusions from.\n   Re-run with a longer window, e.g.:  "
              f"python3 stream_quality.py '' {BAUD} 60")

    # If most traffic isn't frames, SHOW what it is. Readable text here means
    # the S3 is printing/crash-looping; binary noise means a framing problem.
    if unaccounted > len(data) * 0.2 and gaps:
        blob = b"".join(gaps)
        printable = sum(32 <= b < 127 or b in (9, 10, 13) for b in blob)
        pct_text = 100 * printable / max(1, len(blob))
        print(f"\n--- what the unparsed {unaccounted/1000:.0f} KB looks like "
              f"({pct_text:.0f}% printable text) ---")
        sample = blob[:400].decode("ascii", "replace")
        sample = "".join(c if (32 <= ord(c) < 127 or c in "\n\r\t") else "."
                         for c in sample)
        print(sample)
        if len(blob) > 800:
            mid = blob[len(blob)//2:len(blob)//2 + 300].decode("ascii", "replace")
            mid = "".join(c if (32 <= ord(c) < 127 or c in "\n\r\t") else "."
                          for c in mid)
            print("...[middle of capture]...")
            print(mid)
        print("--- end sample ---")
        if pct_text > 50:
            print("-> Mostly TEXT: the S3 is printing something (boot banners, "
                  "panic messages, or stray Serial.print) into the binary "
                  "stream. Find and remove it — or it's crash-looping.")
        else:
            print("-> Mostly BINARY: frame data the parser can't sync to. "
                  "Suspect corrupted length fields or interleaved writes.")

    print()
    print("================ IMAGE ====================")
    if not HAVE_CV:
        # Still save the raw JPEGs — they can be opened in any image viewer,
        # and analysed later on a machine that does have OpenCV.
        os.makedirs(SAMPLE_DIR, exist_ok=True)
        ts = time.strftime("%H%M%S")
        step = max(1, len(frames) // 5)
        for k, (_, jpg) in enumerate(frames[::step][:5]):
            with open(os.path.join(SAMPLE_DIR, f"sample_{ts}_{k}.jpg"), "wb") as f:
                f.write(jpg)
        print("opencv-python / numpy not installed — image metrics skipped.")
        print(f"saved {min(5, len(frames))} raw frames -> {SAMPLE_DIR}")
        print("(open them in any image viewer, or install with:")
        print("   pip install opencv-python numpy   )")
        return

    metrics = [m for m in (image_metrics(j) for _, j in frames) if m]
    if not metrics:
        print("No frame decoded — all corrupt.")
        return
    decode_fail = len(frames) - len(metrics)
    sh = [m["sharpness"] for m in metrics]
    br = [m["brightness"] for m in metrics]
    ct = [m["contrast"] for m in metrics]
    fb = [m["flat_bottom_pct"] for m in metrics]
    h, w = metrics[0]["shape"][:2]
    med_br = statistics.median(br)
    print(f"resolution     : {w}x{h}")
    print(f"decode failures: {decode_fail}")
    print(f"sharpness      : median {statistics.median(sh):.0f}   "
          f"(focus/detail — compare on the SAME scene)")
    print(f"brightness     : median {med_br:.0f} / 255   "
          f"({'DARK — add light' if med_br < 60 else 'ok' if med_br < 200 else 'BRIGHT — may clip'})")
    print(f"contrast       : median {statistics.median(ct):.0f}")
    bad_bottom = sum(1 for v in fb if v > 5)
    print(f"gray-bottom    : {bad_bottom}/{len(metrics)} frames have >5% "
          f"flat bottom rows (truncated captures)")

    # Save evidence
    os.makedirs(SAMPLE_DIR, exist_ok=True)
    ts = time.strftime("%H%M%S")
    step = max(1, len(metrics) // 5)
    for k, m in enumerate(metrics[::step][:5]):
        cv2.imwrite(os.path.join(SAMPLE_DIR, f"sample_{ts}_{k}.jpg"), m["img"])
    print(f"\nsaved {min(5, len(metrics))} sample frames -> {SAMPLE_DIR}")


if __name__ == "__main__":
    main()
