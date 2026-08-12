"""
check_project.py — cross-file consistency check for EDI PIPE CAM
================================================================
Run before flashing, or any time something behaves oddly:

    python3 check_project.py

This project has several constants that MUST agree across files. Every one of
them fails SILENTLY when it drifts — you get plausible behaviour and wrong
results, not an error. This script checks them all in one go.

Exits 1 if anything is wrong, so it can gate a flash.
"""

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
def read(p):
    try:
        return open(os.path.join(HERE, p), errors="ignore").read()
    except OSError:
        return ""

CAM   = read("camera_firmware/camera_firmware.ino")
SCH   = read("THIS ONE/include/StereoCameras.h")
MAIN  = read("THIS ONE/src/main.cpp")
ACT   = read("THIS ONE/src/actuator.cpp")
LEDH  = read("THIS ONE/include/LED.h")
PANEL = read("control_panel_stereo2.py")
CALIB = read("stereo_calibrate.py")

ok, warn, bad = [], [], []


def check(cond, good, failed):
    (ok.append(good) if cond else bad.append(failed))


# ---------------------------------------------------------------- baud
# The single most common failure in this project. Symptom when it drifts:
# TELEMETRY KEEPS WORKING (S3 -> laptop over USB, unaffected) while frame
# counts sit at zero. Changing it means flashing THREE devices: both cameras
# and the S3.
a = re.search(r"Serial\.begin\((\d+)\)", CAM)
b = re.search(r"CAM_BAUD\s*=\s*(\d+)", SCH)
a, b = (a.group(1) if a else "?"), (b.group(1) if b else "?")
check(a == b and a != "?",
      f"baud matches: camera {a} == S3 {b}",
      f"BAUD MISMATCH: camera_firmware.ino={a} but StereoCameras.h={b} "
      f"-> telemetry will work, frames will stay at 0")

# ------------------------------------------------------- orientation
# Calibration describes images as the panel sees them. If these disagree,
# every distance is wrong and nothing warns you.
def orient(src):
    # Leading whitespace allowed: rat88_panel.py defines its fallback inside
    # an `except ImportError:` block, so it is indented.
    r = re.search(r"^\s*ROTATE_DEG\s*=\s*[\[(]\s*(\d+)\s*,\s*(\d+)", src, re.M)
    m = re.search(r"^\s*MIRROR_H\s*=\s*[\[(]\s*(\d+)\s*,\s*(\d+)", src, re.M)
    return (r.groups() if r else None), (m.groups() if m else None)

SCV   = read("showcase_cv.py")
RAT88 = read("rat88_panel.py")

pr, pm = orient(PANEL)
cr, cm = orient(CALIB)

# ROTATE_DEG now lives in FOUR places. rat88_panel.py imports it from
# showcase_cv when cv2 is available, but keeps a literal fallback for the
# no-cv2 path — so the fallback has to agree too, or the picture flips
# depending on whether OpenCV happens to be installed.
for name, src in (("showcase_cv.py", SCV), ("rat88_panel.py", RAT88)):
    r, _ = orient(src)
    if r is None:
        warn.append(f"{name}: no ROTATE_DEG found")
    else:
        check(r == pr,
              f"{name} ROTATE_DEG matches the panel: {r}",
              f"ROTATE_DEG MISMATCH: {name} {r} vs panel {pr}")
    _, mh = orient(src)
    if mh is None:
        warn.append(f"{name}: no MIRROR_H found")
    else:
        check(mh == pm,
              f"{name} MIRROR_H matches the panel: {mh}",
              f"MIRROR_H MISMATCH: {name} {mh} vs panel {pm}")
        # Mirroring only ONE camera gives the two views opposite handedness,
        # so no real-world point can be matched between them at all.
        check(mh[0] == mh[1],
              f"{name} mirrors both cameras or neither",
              f"{name} mirrors only ONE camera {mh} -> stereo impossible")
if "from control_panel_stereo2 import" in CALIB:
    ok.append("calibrator imports orientation from the panel (cannot drift)")
else:
    check(pr == cr,
          f"ROTATE_DEG matches: {pr}",
          f"ROTATE_DEG MISMATCH: panel {pr} vs calibrator {cr} "
          f"-> distances silently wrong")
    if pm != cm:
        bad.append(f"MIRROR_H MISMATCH: panel {pm} vs calibrator {cm}")

# ------------------------------------------------------ LEDC timers
# Channels are PAIRED onto timers (0&1->t0, 2&3->t1, 4&5->t2, 6&7->t3) and
# channels sharing a timer share a frequency. ESP32Servo allocates channels
# dynamically and cannot see raw ledcSetup() calls, so unreserved timers get
# handed out twice -> pressing a servo key drives a motor.
m = re.search(r"ACT_PWM_CH\s*=\s*(\d+)", ACT)
actch = int(m.group(1)) if m else -1
m = re.search(r"int channel\s*=\s*(\d+)", LEDH)
ledch = int(m.group(1)) if m else -1
alloc = re.findall(r"allocateTimer\((\d+)\)", MAIN)

check(len(alloc) >= 1,
      f"servo timers reserved before attach: {alloc}",
      "ESP32PWM::allocateTimer() NOT called in setup() -> ESP32Servo will "
      "hand out channels other code already claimed")
if actch >= 0 and ledch >= 0:
    check(actch // 2 != ledch // 2,
          f"actuator ch{actch} (timer {actch//2}) and LED ch{ledch} "
          f"(timer {ledch//2}) are on different timers",
          f"TIMER CLASH: actuator ch{actch} and LED ch{ledch} share "
          f"timer {actch//2}")
    for name, ch in (("actuator", actch), ("LED", ledch)):
        if str(ch // 2) in alloc:
            bad.append(f"TIMER CLASH: {name} ch{ch} sits on timer {ch//2}, "
                       f"which is reserved for servos")

# -------------------------------------------------------------- pins
pins = dict((n, int(v)) for n, v in
            re.findall(r"int (PIN_\w+|CAM[LR]_RX)\s*=\s*(\d+)", MAIN))
seen = {}
for n, v in pins.items():
    seen.setdefault(v, []).append(n)
clashes = [f"GPIO {v}: {' + '.join(ns)}" for v, ns in seen.items() if len(ns) > 1]
check(not clashes, "no duplicate GPIO assignments",
      "PIN CLASH -> " + "; ".join(clashes))

# ESP32-S3 specifics
RESERVED = {19: "USB D-", 20: "USB D+", 0: "strapping", 3: "strapping",
            45: "strapping", 46: "strapping"}
pin_trouble = False
for v, ns in seen.items():
    if v in RESERVED:
        bad.append(f"GPIO {v} ({'+'.join(ns)}) is {RESERVED[v]} on ESP32-S3")
        pin_trouble = True
    if 26 <= v <= 32:
        bad.append(f"GPIO {v} ({'+'.join(ns)}) is SPI flash/PSRAM on ESP32-S3")
        pin_trouble = True
if not pin_trouble:
    ok.append("no reserved ESP32-S3 pins in use")

# ------------------------------------------------- disparity vs width
# SGBM cannot match the leftmost `numDisparities` columns. If that dead zone
# reaches the centre pixel, the distance readout dies completely.
m = re.search(r"numDisparities=(\d+)", PANEL)
nd = int(m.group(1)) if m else -1
calib_path = os.path.join(HERE, "stereo_calib.npz")
if nd > 0 and os.path.exists(calib_path):
    try:
        import numpy as np
        d = np.load(calib_path)
        fx, bl = float(d["fx"]), float(d["baseline"])
        w = int(d["image_size"][0])
        check(nd < w // 2,
              f"numDisparities {nd} < half-width {w//2} "
              f"(centre pixel usable; nearest {fx*bl/nd/10:.0f} cm)",
              f"numDisparities {nd} blanks the centre pixel of a {w}px-wide "
              f"frame -> distance readout will show nothing")
    except Exception as e:                       # noqa: BLE001
        warn.append(f"could not read stereo_calib.npz ({e})")
elif nd > 0:
    warn.append("no stereo_calib.npz — run stereo_calibrate.py")

# ------------------------------------------------------------ report
print("=" * 68)
for m in ok:
    print(f"  OK    {m}")
for m in warn:
    print(f"  WARN  {m}")
for m in bad:
    print(f"  FAIL  {m}")
print("=" * 68)
print(f"  {len(ok)} ok, {len(warn)} warnings, {len(bad)} failures")
sys.exit(1 if bad else 0)
