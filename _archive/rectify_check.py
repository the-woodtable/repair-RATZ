"""
rectify_check.py — is my stereo calibration actually working?
=============================================================
Loads the stereo_calib.npz you already have and shows the RECTIFIED live pair
with horizontal guide lines. No recalibration, no checkerboard needed.

    python3 rectify_check.py        (close the panels first — one port, one owner)

WHAT TO LOOK FOR
    Pick any feature you can see in both halves — a pipe rim, a cable, a
    corner. It must sit on the SAME green line in both halves.

    lines up      -> rectification is working. Your bonded misalignment has
                     been absorbed and distances are trustworthy.
    consistently
    off by a lot  -> the calibration is bad. Focus both lenses and recapture,
                     with more varied board angles.

WHY THIS IS THE REAL TEST
    Rectification's whole job is to make corresponding points share a row, so
    that the disparity search — which only looks sideways — can find them. If
    rows do not match, SGBM is searching along the wrong line and every
    distance is wrong, no matter how good the RMS looked.

KEYS
    r   toggle raw / rectified, to see what rectification is correcting
    q   quit
"""

import os
import sys
import time

import cv2
import numpy as np
import serial
import serial.tools.list_ports

from stereo_serial import TaggedFrameReader

try:
    from showcase_cv import ROTATE_DEG, MIRROR_H
except Exception:                       # noqa: BLE001
    ROTATE_DEG, MIRROR_H = (90, 270), (1, 1)

HERE = os.path.dirname(os.path.abspath(__file__))
CALIB = os.path.join(HERE, "stereo_calib.npz")
BAUD, SCALE, SPACING = 921600, 2, 20

_FLAG = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
         270: cv2.ROTATE_90_COUNTERCLOCKWISE}


def orient(img, i):
    deg = ROTATE_DEG[i] % 360
    if deg:
        img = cv2.rotate(img, _FLAG[deg])
    if MIRROR_H[i]:
        img = cv2.flip(img, 1)
    return img


def find_port():
    ports = list(serial.tools.list_ports.comports())
    native = [p.device for p in ports if p.vid == 0x303A]
    if native:
        return native[0]
    other = [p.device for p in ports if "bluetooth" not in p.device.lower()]
    return other[0] if other else None


def main():
    if not os.path.exists(CALIB):
        print("No stereo_calib.npz — run stereo_calibrate.py first.")
        return
    d = np.load(CALIB)
    try:
        m1x, m1y, m2x, m2y = d["map1x"], d["map1y"], d["map2x"], d["map2y"]
    except KeyError:
        print("stereo_calib.npz is an old format — re-run stereo_calibrate.py")
        return
    print(f"calibration: rms {float(d['rms']):.3f} px, "
          f"baseline {float(d['baseline']):.1f} mm, "
          f"fx {float(d['fx']):.0f} px, size {tuple(d['image_size'])}")

    port = find_port()
    if port is None:
        print("No serial port found. Is the S3 plugged in?")
        return
    try:
        ser = serial.Serial(port, BAUD, timeout=0.05)
    except serial.SerialException as e:
        print(f"Could not open {port}: {e}  (close the panel first)")
        return

    reader = TaggedFrameReader(ser)
    reader.start()
    last = [None, None]
    rectified = True
    waited = time.time()
    print("\nr = raw/rectified toggle,  q = quit\n")

    while True:
        for i, tag in ((0, b"L"), (1, b"R")):
            j = reader.latest(tag)
            if j is not None:
                img = cv2.imdecode(np.frombuffer(j, np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    last[i] = orient(img, i)

        if last[0] is None or last[1] is None:
            if time.time() - waited > 2.0:
                miss = [n for n, v in (("LEFT", last[0]), ("RIGHT", last[1]))
                        if v is None]
                print(f"waiting for {' and '.join(miss)}...")
                waited = time.time()
            time.sleep(0.02)
            continue

        l, r = last[0], last[1]
        if rectified:
            # The maps were built for the calibration image size. If the
            # camera resolution changed since, remap would silently produce
            # nonsense, so say so rather than showing a lie.
            if l.shape[:2] != m1x.shape[:2]:
                print(f"SIZE MISMATCH: frames are {l.shape[1]}x{l.shape[0]} "
                      f"but the calibration was made at "
                      f"{m1x.shape[1]}x{m1x.shape[0]}. Recalibrate.")
                break
            l = cv2.remap(l, m1x, m1y, cv2.INTER_LINEAR)
            r = cv2.remap(r, m2x, m2y, cv2.INTER_LINEAR)

        view = cv2.hconcat([l, r])
        view = cv2.resize(view, None, fx=SCALE, fy=SCALE,
                          interpolation=cv2.INTER_NEAREST)
        for y in range(0, view.shape[0], SPACING * SCALE):
            cv2.line(view, (0, y), (view.shape[1], y), (0, 220, 0), 1)
        mid = view.shape[1] // 2
        cv2.line(view, (mid, 0), (mid, view.shape[0]), (0, 0, 255), 2)
        cv2.putText(view, "RECTIFIED - features must share a green line"
                    if rectified else "RAW - not rectified",
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 255) if rectified else (0, 165, 255), 2,
                    cv2.LINE_AA)
        cv2.imshow("rectify check   r=raw/rectified   q=quit", view)

        k = cv2.waitKey(30) & 0xFF
        if k == ord("q"):
            break
        if k == ord("r"):
            rectified = not rectified

    reader.stop()
    ser.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
