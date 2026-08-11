"""
align_check.py — settle camera orientation LIVE, before calibrating
====================================================================
Shows both cameras side by side with horizontal guide lines, and lets you
cycle rotation / mirror / left-right with single keys. When it looks right,
press P and it prints the exact lines to paste into the four config files.

    python3 align_check.py          (close the panels first — one port, one owner)

KEYS
    1   rotate LEFT camera  +90
    2   rotate RIGHT camera +90
    b   rotate BOTH +180        (the usual fix for "upside down")
    m   toggle mirror on BOTH   (never one — see below)
    s   swap which pane is left  (display only; tells you what to change)
    p   print settings to paste
    q   quit

WHAT THIS TOOL CAN AND CANNOT FIX
    CAN: the discrete choices — 90-degree rotation, mirroring, which camera
         is "left". Get these right before calibrating.
    CANNOT: a few degrees of roll, or a vertical offset between the cameras.
         Those are what stereoRectify() absorbs, and trying to correct them
         by eye first is wasted effort. Do not chase perfect line-up here.

WHAT "GOOD ENOUGH" LOOKS LIKE
    * both views upright and the same way up as each other
    * covering the LEFT lens darkens the LEFT pane
    * text held up reads correctly (not mirrored) in both
    * a feature sits at roughly — not exactly — the same height in both
      A residual vertical offset is normal and expected. Rectification fixes
      it. If it is wildly different, one camera is rotated wrongly, which the
      guide lines make obvious.
"""

import sys
import time

import cv2
import numpy as np
import serial
import serial.tools.list_ports

from stereo_serial import TaggedFrameReader

BAUD = 921600          # ignored by USB CDC
SCALE = 2
LINE_SPACING = 20      # px between horizontal guides, in source pixels

_FLAG = {90: cv2.ROTATE_90_CLOCKWISE,
         180: cv2.ROTATE_180,
         270: cv2.ROTATE_90_COUNTERCLOCKWISE}

# Live state — start from whatever the project currently uses.
try:
    from showcase_cv import ROTATE_DEG as _R, MIRROR_H as _M
    rot = list(_R)
    mir = list(_M)
except Exception:                       # noqa: BLE001
    rot, mir = [270, 90], [0, 0]
swapped = False


def find_port():
    """The S3's native USB (Espressif VID), not a USB-serial adapter."""
    ports = list(serial.tools.list_ports.comports())
    native = [p.device for p in ports if p.vid == 0x303A]
    if native:
        return native[0]
    other = [p.device for p in ports if "bluetooth" not in p.device.lower()]
    return other[0] if other else None


def orient(img, i):
    deg = rot[i] % 360
    if deg:
        img = cv2.rotate(img, _FLAG[deg])
    if mir[i]:
        img = cv2.flip(img, 1)
    return img


def settings_text():
    return (f"ROTATE_DEG = ({rot[0]}, {rot[1]})\n"
            f"MIRROR_H   = ({mir[0]}, {mir[1]})")


def main():
    global swapped          # 's' rebinds it, so it must be declared here
    port = find_port()
    if port is None:
        print("No serial port found. Is the S3 plugged in?")
        return
    try:
        ser = serial.Serial(port, BAUD, timeout=0.05)
    except serial.SerialException as e:
        print(f"Could not open {port}: {e}")
        print("Close the panel / robot_console / serial monitor first.")
        return
    print(f"Reading {port}\n")
    print(__doc__.split("KEYS")[1].split("WHAT THIS")[0])

    reader = TaggedFrameReader(ser)
    reader.start()
    last = [None, None]
    waited = time.time()

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

        a, b = (last[1], last[0]) if swapped else (last[0], last[1])
        # Pad to equal height so hconcat cannot fail on a mismatched rotation,
        # which is exactly the state you are here to notice.
        h = max(a.shape[0], b.shape[0])
        def pad(x):
            if x.shape[0] == h:
                return x
            out = np.zeros((h, x.shape[1], 3), np.uint8)
            out[:x.shape[0]] = x
            return out
        view = cv2.hconcat([pad(a), pad(b)])
        view = cv2.resize(view, None, fx=SCALE, fy=SCALE,
                          interpolation=cv2.INTER_NEAREST)

        # Horizontal guides: the whole point. A feature should sit at roughly
        # the same height in both halves once rotation is correct.
        for y in range(0, view.shape[0], LINE_SPACING * SCALE):
            cv2.line(view, (0, y), (view.shape[1], y), (0, 200, 0), 1)
        mid = view.shape[1] // 2
        cv2.line(view, (mid, 0), (mid, view.shape[0]), (0, 0, 255), 2)

        cv2.putText(view, f"L rot {rot[0]}  R rot {rot[1]}  mirror {tuple(mir)}"
                          f"{'  [PANES SWAPPED]' if swapped else ''}",
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1,
                    cv2.LINE_AA)
        cv2.imshow("align check  1/2=rotate L/R  b=both180  m=mirror  "
                   "s=swap  p=print  q=quit", view)

        k = cv2.waitKey(30) & 0xFF
        if k == ord("q"):
            break
        elif k == ord("1"):
            rot[0] = (rot[0] + 90) % 360
        elif k == ord("2"):
            rot[1] = (rot[1] + 90) % 360
        elif k == ord("b"):
            rot[0] = (rot[0] + 180) % 360
            rot[1] = (rot[1] + 180) % 360
        elif k == ord("m"):
            # Both or neither, always. One mirrored camera gives the two views
            # opposite handedness and stereo matching becomes impossible.
            v = 0 if mir[0] else 1
            mir[0] = mir[1] = v
        elif k == ord("s"):
            swapped = not swapped
        elif k == ord("p"):
            print("\n" + "=" * 58)
            print(settings_text())
            print("paste into ALL FOUR:")
            print("  showcase_cv.py   rat88_panel.py")
            print("  control_panel_stereo2.py   stereo_calibrate.py")
            if swapped:
                print("\nPANES ARE SWAPPED in this preview. To make it real,")
                print("swap CAML_RX / CAMR_RX in THIS ONE/src/main.cpp and")
                print("reflash the S3 — do NOT also swap the wires.")
            print("=" * 58 + "\n")

    print("\nfinal settings:")
    print(settings_text())
    reader.stop()
    ser.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
