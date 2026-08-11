"""
EDI PIPE CAM — stereo calibration (FISHEYE lens version)
---------------------------------------------------------
Run ONCE after both cameras are rigidly mounted. If a camera is ever
remounted or bumped, recalibrate.

    pip install pyserial opencv-python numpy
    python stereo_calibrate.py

Needs a printed checkerboard (default 10x7 INNER corners, 19.5 mm squares —
edit below to match yours; measure the printed square size!).

WHY FISHEYE, NOT THE STANDARD MODEL:
Your camera lenses are wide-angle (120-160 degree class). The standard
pinhole model (cv2.calibrateCamera / cv2.stereoCalibrate) assumes mild,
low-order lens distortion and does NOT fit a lens this wide well,
especially toward the frame edges -- you can get a deceptively low
reported RMS error while the actual rectification is still warped or
misaligned. cv2.fisheye.* uses a distortion model designed for this.

Keys (with the preview window focused):
    SPACE  capture a pair (hold the board still, visible in BOTH views)
    c      calibrate using captured pairs (need >= 15 for fisheye -- it
           has more distortion parameters to fit than the standard model,
           so it needs more views, from varied angles/distances) and
           save stereo_calib.npz
    q      quit
"""

import os
import time

import cv2
import numpy as np
import serial
import serial.tools.list_ports

from stereo_serial import TaggedFrameReader

# ------------------- Settings -------------------
PORT = None               # None = auto-detect (cross-platform, by VID); or force e.g. "COM7"
BAUD = 921600
# INNER corners, not squares: an 11x8 square board has 10x7 inner corners
# (the crossings where four squares meet — the outer edges don't count).
# Get this wrong and findChessboardCorners simply never matches, with no
# error message; you just press SPACE forever and nothing is captured.
BOARD = (10, 7)           # inner corners (cols, rows) — 11x8 square board

# SETS THE SCALE OF EVERY DISTANCE YOU WILL EVER MEASURE. Measure a printed
# square with a ruler; don't trust what the PDF claimed, printers rescale.
# If this is wrong by 10%, every reported distance is wrong by 10%.
SQUARE_MM = 19.5          # printed square size in mm — MEASURE IT
OUT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "stereo_calib.npz")

# Fisheye calibration needs more views than the standard model (more
# distortion parameters to fit reliably). 15 is a practical floor;
# more, and more varied, is always better.
MIN_PAIRS = 15

# How much image to keep when rectifying, fisheye's equivalent of the
# standard model's alpha. 0.0 = crop to only fully-valid pixels (narrower
# effective FOV, no black borders). 1.0 = keep every pixel (full FOV, black
# wedges where one camera had no data). 0.5 is a middle ground.
BALANCE = 0.5

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
    """Pick the S3's own native USB port, on any OS.

    Matches by VID (USB vendor ID), not by device name string -- device
    names differ by OS (COM7 on Windows, /dev/cu.usbmodem101 on Mac,
    /dev/ttyACM0 on Linux) but the VID is a property of the USB hardware
    itself and is identical everywhere.

    0x303A = Espressif's own vendor ID -- the S3's NATIVE USB CDC/JTAG.
    Common USB-serial ADAPTER chips (what you flash the cameras through,
    not the S3 itself): CH340 = 0x1A86, FTDI = 0x0403, CP210x = 0x10C4.
    """
    if PORT:
        return PORT

    ESPRESSIF_VID = 0x303A
    ADAPTER_VIDS = {0x1A86, 0x0403, 0x10C4}

    ports = list(serial.tools.list_ports.comports())
    native = [p.device for p in ports if p.vid == ESPRESSIF_VID]
    adapter = [p.device for p in ports if p.vid in ADAPTER_VIDS]

    if not native and not adapter:
        return None
    if not native:
        print("No native Espressif USB port -- the S3 may not be plugged in.")
        print(f"Falling back to the USB-serial adapter {adapter[0]}. If you "
              "are trying to FLASH a camera, close this panel first.")
        return adapter[0]
    if len(native) > 1 or adapter:
        print("Serial devices seen:", ", ".join(p.device for p in ports))
        print(f"Using {native[0]} — set PORT at the top to override.")
    return native[0]


def _to_fisheye_points(obj_pts, pts_l, pts_r):
    """cv2.fisheye.* wants each view's points as float64 arrays shaped
    (1, N, 3) for object points and (1, N, 2) for image points -- NOT the
    (N, 1, 3)/(N, 1, 2) shape cv2.calibrateCamera and findChessboardCorners
    give you. Reshape here once rather than getting cryptic assertion
    errors deep inside OpenCV's C++ layer."""
    n = BOARD[0] * BOARD[1]
    obj_fe = [o.reshape(1, n, 3).astype(np.float64) for o in obj_pts]
    l_fe = [p.reshape(1, n, 2).astype(np.float64) for p in pts_l]
    r_fe = [p.reshape(1, n, 2).astype(np.float64) for p in pts_r]
    return obj_fe, l_fe, r_fe


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
            if len(obj_pts) < MIN_PAIRS:
                print(f"only {len(obj_pts)} pairs — fisheye needs at least "
                      f"{MIN_PAIRS}, from varied angles/distances")
                continue
            print("calibrating (fisheye model)...")

            obj_fe, l_fe, r_fe = _to_fisheye_points(obj_pts, pts_l, pts_r)

            # Not every opencv-python build exposes all three constants under
            # cv2.fisheye (varies by version/build) -- look each up
            # defensively instead of assuming, so a missing one degrades
            # gracefully (flag just isn't set) rather than crashing.
            fe_flags = 0
            for name in ("CALIB_RECOMPUTE_EXTRINSIC", "CALIB_CHECK_COND",
                        "CALIB_FIX_SKEW"):
                fe_flags |= getattr(cv2.fisheye, name, 0)
            crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    100, 1e-6)

            k1 = np.zeros((3, 3))
            d1 = np.zeros((4, 1))
            k2 = np.zeros((3, 3))
            d2 = np.zeros((4, 1))
            try:
                _, k1, d1, _, _ = cv2.fisheye.calibrate(
                    obj_fe, l_fe, img_size, k1, d1, flags=fe_flags,
                    criteria=crit)
                _, k2, d2, _, _ = cv2.fisheye.calibrate(
                    obj_fe, r_fe, img_size, k2, d2, flags=fe_flags,
                    criteria=crit)
            except cv2.error as e:
                print(f"fisheye single-camera calibration failed: {e}")
                print("Usually means one or more captured pairs has a bad "
                      "corner detection (board too tilted/close to the "
                      "extreme edge). Try discarding recent captures and "
                      "recapturing from more moderate angles.")
                continue

            try:
                rms, k1, k2, d1, d2, r, t = cv2.fisheye.stereoCalibrate(
                    obj_fe, l_fe, r_fe, k1, d1, k2, d2, img_size,
                    flags=getattr(cv2.fisheye, "CALIB_FIX_INTRINSIC", 0),
                    criteria=crit)
            except cv2.error as e:
                print(f"fisheye stereo calibration failed: {e}")
                continue

            baseline = float(np.linalg.norm(t))
            print(f"RMS error: {rms:.3f} px (want < ~0.5)")
            print(f"baseline: {baseline:.1f} mm (sanity-check against ruler!)")

            # Precompute the rectification maps so the panel can just load
            # and use them (it must not have to redo stereoRectify).
            r1, r2, p1, p2, _ = cv2.fisheye.stereoRectify(
                k1, d1, k2, d2, img_size, r, t,
                flags=cv2.CALIB_ZERO_DISPARITY, balance=BALANCE, fov_scale=1.0)
            map1x, map1y = cv2.fisheye.initUndistortRectifyMap(
                k1, d1, r1, p1, img_size, cv2.CV_32FC1)
            map2x, map2y = cv2.fisheye.initUndistortRectifyMap(
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
                  f"corrected. Raise BALANCE (now {BALANCE}) to keep more "
                  "image, lower it toward 0 to crop them away.")

            np.savez(OUT_FILE,
                     # raw calibration (kept for reference / recomputation)
                     K1=k1, D1=d1, K2=k2, D2=d2, R=r, T=t,
                     image_size=np.array(img_size),
                     model="fisheye",     # so future tooling can tell at a glance
                     # ready-to-use rectification + depth constants
                     # (SAME KEY NAMES as the standard-model version, so
                     # control_panel_stereo2.py's StereoCalib class needs
                     # no changes at all)
                     map1x=map1x, map1y=map1y, map2x=map2x, map2y=map2y,
                     fx=fx, baseline=baseline, rms=float(rms))
            print(f"saved {OUT_FILE} — put it next to control_panel_stereo2.py")

        elif key == ord("q"):
            break

    reader.stop()
    ser.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()