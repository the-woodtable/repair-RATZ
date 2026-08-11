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
# INNER corners, not squares: an 11x8 square board has 10x7 inner corners
# (the crossings where four squares meet — the outer edges don't count).
# Get this wrong and findChessboardCorners simply never matches, with no
# error message; you just press SPACE forever and nothing is captured.
BOARD = (10, 7)           # inner corners (cols, rows) — 11x8 square board

# SETS THE SCALE OF EVERY DISTANCE YOU WILL EVER MEASURE. Measure a printed
# square with a ruler; don't trust what the PDF claimed, printers rescale.
# If this is wrong by 10%, every reported distance is wrong by 10%.
# Self-check: the baseline this script prints must match a ruler measurement
# of the gap between your two lenses. If it doesn't, this number is why.
# 19.5 measured with a ruler on the actual printout. The first run used 15.0
# and produced a 95.9 mm baseline for a rig whose lenses are 125 mm apart —
# wrong by exactly 19.5/15.0. That is the self-check working.
SQUARE_MM = 19.5          # printed square size in mm — MEASURE IT
OUT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "stereo_calib.npz")

# How much image to keep when rectifying. This matters a lot when the two
# cameras are NOT well aligned — which ours are not, being bonded 180 degrees
# apart with a few degrees of residual roll.
#
#   0.0  crop to only fully-valid pixels. No black borders, but a badly
#        aligned pair loses a LOT of field of view to the crop.
#   1.0  keep every pixel. Nothing is thrown away, but the edges have black
#        wedges where one camera had no data.
#   0.5  a middle ground: most of the view, modest borders.
#
# We were on 0.0, which is why the effective FOV came out much narrower than
# the 120-degree lenses should give. Black borders are harmless for viewing —
# SGBM simply finds no match there and reports no distance, exactly as it
# would off the edge of the frame.
RECTIFY_ALPHA = 0.5


# MUST match ROTATE_DEG in control_panel_stereo2.py — (left, right).
# Calibration describes the images as the panel will see them; if the two
# files disagree, every distance the panel reports is wrong.
ROTATE_DEG = (90, 270)   # swapped with the L/R id swap in main.cpp

# Horizontal mirror per camera, 0 or 1. Applied AFTER rotation, so it means
# "flip what you are looking at left-to-right".
#
# A DIFFERENT operation from rotation: rotations preserve handedness, a mirror
# reverses it, so no combination of rotations can substitute for one.
#
# Both cameras must match. Mirroring only one gives the two views opposite
# handedness and no real-world point can be matched between them, which makes
# stereo depth impossible rather than merely inaccurate.
MIRROR_H = (1, 1)

_ROTATE_FLAG = {90: cv2.ROTATE_90_CLOCKWISE,
                180: cv2.ROTATE_180,
                270: cv2.ROTATE_90_COUNTERCLOCKWISE}


# Corrupt-frame rejection. A torn frame is worse here than in a live view:
# findChessboardCorners can still "succeed" on damaged pixels and feed the
# solver garbage points, which quietly ruins the calibration.
MAX_FLAT_BOTTOM = 0.05
MAX_COLOUR_JUMP = 45.0
rejected = [0, 0]          # per camera, reported while you capture


def decode(jpeg, cam):
    """cam: 0 = left, 1 = right (they may be mounted differently).

    Returns None for a corrupt frame so the caller keeps its last good one.
    """
    # Decoded in COLOUR even though corner-finding wants grey, because the
    # colour-band test needs the channels. Converted to grey at the end.
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        rejected[cam] += 1
        return None

    # CHECK BEFORE ROTATING — JPEG damage lands on the bottom rows in sensor
    # orientation, and a 90/270 rotation moves it to a side edge where a
    # bottom-rows test cannot see it.
    gray_std = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).std(axis=1)
    flat = 0
    for v in gray_std[::-1]:
        if v < 2.0:
            flat += 1
        else:
            break
    if flat / len(gray_std) > MAX_FLAT_BOTTOM:
        rejected[cam] += 1
        return None

    rows = img.reshape(img.shape[0], -1, 3).mean(axis=1)
    if len(rows) >= 8:
        chroma = np.stack([rows[:, 0] - rows[:, 1],
                           rows[:, 2] - rows[:, 1]], 1)
        diffs = np.abs(np.diff(chroma[3:-3], axis=0)).sum(axis=1)
        if len(diffs) and diffs.max() > MAX_COLOUR_JUMP:
            rejected[cam] += 1
            return None

    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    deg = ROTATE_DEG[cam] % 360
    if deg:
        img = cv2.rotate(img, _ROTATE_FLAG[deg])
    if MIRROR_H[cam]:
        img = cv2.flip(img, 1)
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
    waiting_since = time.time()
    print("SPACE=capture pair, c=calibrate+save, q=quit")
    print("(the preview window only appears once BOTH cameras have sent a "
          "frame)")

    while True:
        jl, jr = reader.latest(b"L"), reader.latest(b"R")
        # Only replace the held frame when decode SUCCEEDS. Assigning the
        # result directly would blank the view on every corrupt frame, which
        # is exactly the flicker we are trying to remove.
        if jl is not None:
            d = decode(jl, 0)
            if d is not None:
                last_l = d
        if jr is not None:
            d = decode(jr, 1)
            if d is not None:
                last_r = d
        if last_l is None or last_r is None:
            # Say WHICH camera we're waiting on. Previously this loop was
            # silent, so a dead channel looked identical to a script that
            # had hung — no window, no message, nothing to act on.
            now = time.time()
            if now - waiting_since > 2.0:
                missing = []
                if last_l is None:
                    missing.append("LEFT")
                if last_r is None:
                    missing.append("RIGHT")
                print(f"waiting for {' and '.join(missing)} "
                      f"— no frames yet. Is the S3 running? "
                      f"Is the panel still open holding the port?")
                waiting_since = now
            time.sleep(0.02)
            continue

        img_size = (last_l.shape[1], last_l.shape[0])
        view = cv2.hconcat([last_l, last_r])
        cv2.putText(view, f"pairs: {len(obj_pts)}   rejected L{rejected[0]} "
                          f"R{rejected[1]}", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255, 1)
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
                flags=cv2.CALIB_ZERO_DISPARITY, alpha=RECTIFY_ALPHA)
            map1x, map1y = cv2.initUndistortRectifyMap(
                k1, d1, r1, p1, img_size, cv2.CV_32FC1)
            map2x, map2y = cv2.initUndistortRectifyMap(
                k2, d2, r2, p2, img_size, cv2.CV_32FC1)
            fx = float(p1[0, 0])          # rectified focal length, pixels

            # ---- VERIFY THE RECTIFICATION ----
            # Corresponding points must land on the SAME ROW after rectifying.
            # Drawing horizontal lines across both views is the standard way
            # to check: pick any feature, and it should sit on the same line
            # in both halves. If it does, the mount misalignment has been
            # fully absorbed no matter how crooked the cameras are.
            rl = cv2.remap(last_l, map1x, map1y, cv2.INTER_LINEAR)
            rr = cv2.remap(last_r, map2x, map2y, cv2.INTER_LINEAR)
            check = cv2.cvtColor(cv2.hconcat([rl, rr]), cv2.COLOR_GRAY2BGR)
            for y in range(0, check.shape[0], 20):
                cv2.line(check, (0, y), (check.shape[1], y), (0, 255, 0), 1)
            cv2.line(check, (rl.shape[1], 0), (rl.shape[1], check.shape[0]),
                     (0, 0, 255), 2)
            cv2.imshow("RECTIFIED - a feature must sit on the SAME green line "
                       "in both halves", check)
            print("A rectification check window opened. Pick any feature and "
                  "confirm it is on the same green line in both halves.")
            print("Black borders are normal: that is the misalignment being "
                  f"corrected. Raise RECTIFY_ALPHA (now {RECTIFY_ALPHA}) to "
                  "keep more image, lower it toward 0 to crop them away.")

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
