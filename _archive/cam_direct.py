"""
cam_direct.py — view ONE ESP32-CAM wired straight to the laptop
================================================================
Bypasses the S3 entirely. This is the test that cracked the frame-corruption
problem once already: if the camera looks perfect here but bad through the
S3, the camera is fine and the damage is downstream.

WIRING (USB-serial adapter, CH340 / FTDI / CP2102):
    camera U0T  ->  adapter RX
    camera GND  ->  adapter GND      <- REQUIRED, and keep it short
    camera 5V   ->  adapter 5V       (or an external supply; the camera can
                                      draw ~250 mA peak and some adapters sag)
    adapter TX  ->  not needed; nothing is sent to the camera

    IO0 must be FLOATING to run. IO0 -> GND is flash mode only.

WIRE FORMAT
    The camera alone sends:   0xAA 0x55 | len(4, LE) | JPEG
    Through the S3 it gains a camera-id byte, which is why stream_quality.py
    and the panel cannot read a camera directly. Identity normally comes from
    which S3 UART the camera is wired to, not from the camera itself.

USAGE
    python3 cam_direct.py                # auto-detect adapter, 460800
    python3 cam_direct.py 921600         # different baud
    python3 cam_direct.py /dev/cu.usbserial-1140 460800

    q = quit    s = save the current frame

Baud MUST match Serial.begin() in camera_firmware.ino.
"""

import os
import sys
import time

import cv2
import numpy as np
import serial
import serial.tools.list_ports

MAGIC = b"\xAA\x55"
MAX_FRAME = 300_000
MIN_FRAME = 512
OUT_DIR = os.path.expanduser("~/Desktop/30.007/pipe_cam_data/cam_direct")


def find_adapter():
    """Prefer a USB-SERIAL ADAPTER here — the opposite of the panel.

    The panel wants the S3 (usbmodem / ttyACM). This script talks to a camera
    through a CH340/FTDI, which enumerates as usbserial / ttyUSB.
    """
    ports = [p.device for p in serial.tools.list_ports.comports()]
    adapter = [d for d in ports
               if any(k in d.lower() for k in ("usbserial", "ttyusb"))]
    if adapter:
        if len(adapter) > 1:
            print("Multiple adapters:", ", ".join(adapter), "— using", adapter[0])
        return adapter[0]
    other = [d for d in ports if "bluetooth" not in d.lower()]
    if other:
        print(f"No USB-serial adapter found; falling back to {other[0]}")
        return other[0]
    return None


def flat_bottom_frac(img):
    """Fraction of bottom rows that are flat — the signature of a truncated
    capture. Checked in SENSOR orientation; no rotation is applied here."""
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    rows_flat = 0
    for y in range(g.shape[0] - 1, -1, -1):
        if g[y].std() < 2.0:
            rows_flat += 1
        else:
            break
    return rows_flat / g.shape[0]


def main():
    args = sys.argv[1:]
    port = next((a for a in args if not a.isdigit()), None) or find_adapter()
    baud = next((int(a) for a in args if a.isdigit()), 460800)

    if port is None:
        print("No serial port found. Is the adapter plugged in?")
        return
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"Could not open {port}: {e}")
        print("If busy: close the panel, robot_console, or any serial monitor.")
        return

    print(f"Reading {port} at {baud} baud.  q = quit, s = save\n")
    os.makedirs(OUT_DIR, exist_ok=True)

    sync = bytearray()
    good = truncated = nodecode = badhdr = 0
    bytes_total = bytes_frames = 0
    t0 = time.time()
    last_report = t0
    frame = None

    try:
        while True:
            b = ser.read(1)
            if not b:
                if time.time() - last_report > 2.0:
                    print("no data — check wiring, IO0 floating, and that the "
                          f"camera is running at {baud} baud")
                    last_report = time.time()
                continue
            bytes_total += 1
            sync += b
            if len(sync) > 2:
                del sync[0]
            if bytes(sync) != MAGIC:
                continue
            sync.clear()

            head = ser.read(4)
            bytes_total += len(head)
            if len(head) < 4:
                continue
            length = int.from_bytes(head, "little")
            if not (MIN_FRAME <= length <= MAX_FRAME):
                badhdr += 1
                continue

            body = bytearray()
            while len(body) < length:
                chunk = ser.read(length - len(body))
                if not chunk:
                    break
                body.extend(chunk)
            bytes_total += len(body)
            if len(body) < length or body[:2] != b"\xFF\xD8":
                badhdr += 1
                continue

            img = cv2.imdecode(np.frombuffer(bytes(body), np.uint8),
                               cv2.IMREAD_COLOR)
            if img is None:
                nodecode += 1
                continue
            if flat_bottom_frac(img) > 0.05:
                truncated += 1
                continue

            good += 1
            bytes_frames += length
            frame = img

            cv2.imshow("ESP32-CAM direct  (q=quit, s=save)",
                       cv2.resize(img, None, fx=2, fy=2,
                                  interpolation=cv2.INTER_NEAREST))
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s") and frame is not None:
                p = os.path.join(OUT_DIR, f"frame_{int(time.time())}.png")
                cv2.imwrite(p, frame)
                print("saved", p)

            now = time.time()
            if now - last_report >= 5.0:
                el = now - t0
                total = good + truncated + nodecode + badhdr
                bad_pct = 100.0 * (total - good) / total if total else 0.0
                unacc = 100.0 * (1 - bytes_frames / bytes_total) if bytes_total else 0
                print(f"{el:5.0f}s  {good/el:5.1f} fps   "
                      f"good {good:5d}  bad {total-good:4d} ({bad_pct:4.1f}%)   "
                      f"trunc {truncated:4d}  nodecode {nodecode:3d}  "
                      f"badhdr {badhdr:3d}   "
                      f"{bytes_frames/el/1000:5.1f} KB/s   "
                      f"unaccounted {unacc:3.0f}%")
                last_report = now
    except KeyboardInterrupt:
        pass

    ser.close()
    cv2.destroyAllWindows()
    el = max(time.time() - t0, 1e-9)
    total = good + truncated + nodecode + badhdr
    print(f"\n{good} good frames in {el:.0f}s = {good/el:.1f} fps, "
          f"{100.0*(total-good)/total if total else 0:.1f}% bad")
    print("Clean here but bad through the S3 -> the camera is fine and the "
          "problem is the S3 link or its wiring.")


if __name__ == "__main__":
    main()
