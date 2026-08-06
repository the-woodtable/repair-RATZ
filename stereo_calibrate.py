"""
EDI PIPE CAM — stereo calibration
---------------------------------
Run ONCE after both cameras are rigidly mounted. If a camera is ever
remounted or bumped, recalibrate.

    pip install pyserial opencv-python numpy
    python stereo_calibrate.py

Needs a printed checkerboard (default 9x6 INNER corners, 25 mm squares —
edit below to match yours; measure the printed square size!).

Keys (with the preview window focused):
    SPACE  capture a pair (hold the board still, visible in BOTH views)
    c      calibrate using captured pairs (need >= 10, from varied
           angles/distances) and save stereo_calib.npz
    q      quit
"""

import os
import time

import cv2
import numpy as np
import serial

from stereo_serial import TaggedFrameReader

# ------------------- Settings -------------------
PORT = None               # None = auto-detect; or set e.g. "/dev/cu.usbmodem101"
BAUD = 921600
BOARD = (9, 6)            # inner corners (cols, rows)
SQUARE_MM = 25.0          # printed square size in mm — MEASURE IT
OUT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "stereo_calib.npz")


# MUST match ROTATE_DEG in control_panel_stereo2.py — (left, right).
# Calibration describes the images as the panel will see them; if the two
# files disagree, every distance the panel reports is wrong.
ROTATE_DEG = (270, 90)
_ROTATE_FLAG = {90: cv2.ROTATE_90_CLOCKWISE,
                180: cv2.ROTATE_180,
                270: cv2.ROTATE_90_COUNTERCLOCKWISE}


def decode(jpeg, cam):
    """cam: 0 = left, 1 = right (they may be mounted differently)."""
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
    deg = ROTATE_DEG[cam] % 360
    if img is not None and deg:
        img = cv2.rotate(img, _ROTATE_FLAG[deg])
    return img


def find_port():
    if PORT:
        return PORT
    from serial.tools import list_ports
    candidates = [p.device for p in list_ports.comports()
                  if any(k in p.device.lower()
                         for k in ("usbmodem", "usbserial", "ttyacm", "ttyusb"))]
    return candidates[0] if candidates else None


def main():
    port = find_port()
    if port is None:
        print("No ESP32 serial port found. Is the S3 plugged in via USB?")
        return
    print(f"Connected to {port}")
    ser = serial.Serial(port, BAUD, timeout=0.05)
    reader = TaggedFrameReader(ser)
    reader.start()

    # Object points template (checkerboard corners in mm, Z=0 plane)
    objp = np.zeros((BOARD[0] * BOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:BOARD[0], 0:BOARD[1]].T.reshape(-1, 2) * SQUARE_MM

    obj_pts, pts_l, pts_r = [], [], []
    img_size = None
    last_l = last_r = None
    print("SPACE=capture pair, c=calibrate+save, q=quit")

    while True:
        jl, jr = reader.latest(b"L"), reader.latest(b"R")
        if jl is not None:
            last_l = decode(jl, 0)
        if jr is not None:
            last_r = decode(jr, 1)
        if last_l is None or last_r is None:
            time.sleep(0.02)
            continue

        img_size = (last_l.shape[1], last_l.shape[0])
        view = cv2.hconcat([last_l, last_r])
        cv2.putText(view, f"pairs: {len(obj_pts)}", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255, 2)
        cv2.imshow("L | R  (SPACE=capture, c=calibrate, q=quit)", view)
        key = cv2.waitKey(30) & 0xFF

        if key == ord(" "):
            ok_l, cl = cv2.findChessboardCorners(last_l, BOARD)
            ok_r, cr = cv2.findChessboardCorners(last_r, BOARD)
            if ok_l and ok_r:
                crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                        30, 0.001)
                cl = cv2.cornerSubPix(last_l, cl, (5, 5), (-1, -1), crit)
                cr = cv2.cornerSubPix(last_r, cr, (5, 5), (-1, -1), crit)
                obj_pts.append(objp)
                pts_l.append(cl)
                pts_r.append(cr)
                print(f"captured pair {len(obj_pts)}")
            else:
                print("checkerboard not found in both views — adjust and retry")

        elif key == ord("c"):
            if len(obj_pts) < 10:
                print(f"only {len(obj_pts)} pairs — capture at least 10")
                continue
            print("calibrating...")
            _, k1, d1, _, _ = cv2.calibrateCamera(obj_pts, pts_l, img_size, None, None)
            _, k2, d2, _, _ = cv2.calibrateCamera(obj_pts, pts_r, img_size, None, None)
            rms, k1, d1, k2, d2, r, t, _, _ = cv2.stereoCalibrate(
                obj_pts, pts_l, pts_r, k1, d1, k2, d2, img_size,
                flags=cv2.CALIB_FIX_INTRINSIC)
            baseline = float(np.linalg.norm(t))
            print(f"RMS error: {rms:.3f} px (want < ~0.5)")
            print(f"baseline: {baseline:.1f} mm (sanity-check against ruler!)")

            # Precompute the rectification maps so the panel can just load
            # and use them (it must not have to redo stereoRectify).
            r1, r2, p1, p2, _, _, _ = cv2.stereoRectify(
                k1, d1, k2, d2, img_size, r, t,
                flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
            map1x, map1y = cv2.initUndistortRectifyMap(
                k1, d1, r1, p1, img_size, cv2.CV_32FC1)
            map2x, map2y = cv2.initUndistortRectifyMap(
                k2, d2, r2, p2, img_size, cv2.CV_32FC1)
            fx = float(p1[0, 0])          # rectified focal length, pixels

            np.savez(OUT_FILE,
                     # raw calibration (kept for reference / recomputation)
                     K1=k1, D1=d1, K2=k2, D2=d2, R=r, T=t,
                     image_size=np.array(img_size),
                     # ready-to-use rectification + depth constants
                     map1x=map1x, map1y=map1y, map2x=map2x, map2y=map2y,
                     fx=fx, baseline=baseline, rms=float(rms))
            print(f"saved {OUT_FILE} — put it next to control_panel_stereo.py")

        elif key == ord("q"):
            break

    reader.stop()
    ser.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
