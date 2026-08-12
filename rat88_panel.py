#!/usr/bin/env python3
"""
rat #88 - showcase control panel (PySide6)

Speaks the SAME protocol as control_panel_stereo2.py, so it is a drop-in
alternative front end. Keep the old panel for diagnostics and recording; use
this one when people are watching.

    pip3 install PySide6
    pip3 install pyserial                  # else SIM mode only
    pip3 install opencv-python ultralytics # else no crack overlays / distance

    python3 rat88_panel.py                       # auto-detect port
    python3 rat88_panel.py --sim                 # no hardware, animated feed
    python3 rat88_panel.py --port /dev/cu.usbmodemXXXX
    python3 rat88_panel.py --no-cv               # skip YOLO+SGBM if it's slow
    python3 rat88_panel.py --conf 0.55           # detection threshold
    python3 rat88_panel.py --list                # show ports and exit

Crack detection runs on the LEFT camera only and distance comes from the
stereo disparity map, so the right pane stays cheap. See showcase_cv.py.

Wire format in  (from stereo_serial.py / main.cpp):
    0xAA 0x55 | ID(1) | uint32 LE length | payload
    ID 0x00 or 'L' = left camera JPEG
    ID 0x01 or 'R' = right camera JPEG
    ID 0x02        = telemetry (JSON-ish text)

Wire format out (single characters, no terminator - what main.cpp listens for):
    F/B/S      drive forward / back / stop
    D/R/X      lining actuator deploy / retract / stop
    H/A/U      hook servo    180 / 90 / 0
    T/Y        spool servo   180 / 90
    L/O        LED on / off
    Z          zero odometry
    '0'-'9'    lamp brightness, mapped to 0-255 in firmware

Three different kinds of hardware, so three different kinds of control:
    DRIVE     Stepper::setDrive has NO timeout. It runs until told 'S', so the
              arrows are press-and-hold and send 'S' on release, on
              mouse-leave, and when the window loses focus.
    LINING    An actuator - a motor that runs. Actuator::update() cuts it
              after MAX_RUN_MS (10s, THIS ONE/include/Actuator.h). One click
              starts a move; the firmware ends it.
    HOOK,     Servos. setAngle drives them to a position and holds. Nothing
    SPOOL     to stop and no timer needed; one click parks them.
"""

import argparse
import math
import os
import queue
import re
import struct
import sys
import threading
import time
from pathlib import Path

from PySide6.QtCore import (QBuffer, QObject, QPoint, QRect, QSize, Qt,
                            QTimer, Signal)
from PySide6.QtGui import (QColor, QFont, QIcon, QImage, QPainter,
                           QPen, QPixmap, QTransform)
from PySide6.QtWidgets import (QApplication, QDialog, QGridLayout, QLabel,
                               QPushButton, QScrollArea, QSlider, QVBoxLayout,
                               QWidget)

try:
    import auto_sequence as auto
    HAVE_AUTO = True
except ImportError as _aexc:         # panel still runs, just manual-only
    HAVE_AUTO = False
    AUTO_IMPORT_ERROR = str(_aexc)

try:
    import serial
    from serial.tools import list_ports
    HAVE_SERIAL = True
except ImportError:
    HAVE_SERIAL = False

# Per-camera rotation, (left, right). Taken from showcase_cv so there is one
# source of truth when OpenCV is available; the literal is only a fallback for
# the no-cv2 path, which is exactly the case that needs it most.
# KEEP THE FALLBACK IN SYNC WITH showcase_cv.py — check_project.py verifies it.
try:
    from showcase_cv import ROTATE_DEG, MIRROR_H
except Exception:                    # noqa: BLE001 - cv2 missing, etc.
    ROTATE_DEG = (90, 270)
    MIRROR_H = (1, 1)

try:
    import numpy as np
    import cv2
    import showcase_cv as scv
    HAVE_CV = True
    CV_IMPORT_ERROR = None
except ImportError as _exc:          # panel still runs, just without overlays
    HAVE_CV = False
    CV_IMPORT_ERROR = str(_exc)


# ---------------------------------------------------------------------------
# ASSETS - every picture the panel draws.
#
# Paths are relative to THIS FILE, not to wherever you launched the terminal
# from, so `python3 rat88_panel.py` works from any directory. PNG with
# transparency is what you want; the alpha channel is what makes the
# odd-shaped buttons click correctly. SVG also loads.
#
# All of these are required except the "_on" and "_down" variants, which are
# the extra states: "_on" is the lit look for a button in a selection group,
# "_down" is the held look for a press-and-hold button. Leave one None and the
# panel generates that state by fading or tinting the base image. Missing
# files are reported all at once at startup by check_assets().
# ---------------------------------------------------------------------------
ASSETS = {
    "background":     "buttons/21background.png",       # plain 4:3 plate

    # Decorations. Drawn behind every control, so they may overlap freely.
    # The header art is the rat and its footprints only; the title text is
    # still drawn by the panel on top of it.
    "header":         "buttons/18ratz_header.png",
    "mouse_top":      "buttons/20cheeze.png",           # top right, with heart
    "mouse_bottom":   "buttons/19say_cheeze.png",       # bottom left, with camera

    "bezel":          "buttons/17cam_background.png",   # frame round the video
    "arrow_up":       "buttons/01fwd_unpressed.png",
    "arrow_up_down":  "buttons/03fwd_pressed.png",      # shown while held
    "arrow_down":     "buttons/02bwd_unpressed.png",
    "arrow_down_down": "buttons/04bwd_pressed.png",
    "cheese":         "buttons/06cheese_slider.png",    # slider handle
    "groove":         "buttons/24slider_nosuns.png",    # vertical slider bar
    "sun_max":        "buttons/22sun_max.png",          # big sun, top
    "sun_min":        "buttons/23sun_min.png",          # small sun, bottom

    # Lining pair. "_on" is the selected/lit artwork.
    "can":            "buttons/07lining_compressed.png",
    "can_on":         "buttons/08lining_deployed.png",
    # Automation and utilities. The spool buttons used to live here; the
    # spool servo is still reachable from the keyboard (T / Y).
    "start":          "buttons/27start.png",
    "stop":           "buttons/28stop.png",
    "home":           "buttons/29home.png",
    "folder":         "buttons/30folder.png",
    "distance":       "buttons/cm_travelled.png",   # click to zero

    # Hook positions, named the way you name them. HOOK_POSITIONS below says
    # which servo command each one sends.
    "pos_hook":         "buttons/12hook_upright_unpressed.png",
    "pos_hook_on":      "buttons/11hook_upright_pressed.png",
    "pos_assembly":     "buttons/14hook_side_unpressed.png",
    "pos_assembly_on":  "buttons/13hook_side_pressed.png",
    "pos_deploy":       "buttons/16hook_down_unpressed.png",
    "pos_deploy_on":    "buttons/15hook_down_pressed.png",
}

HERE = Path(__file__).resolve().parent


def asset_path(p: str) -> str:
    q = Path(p)
    return str(q if q.is_absolute() else HERE / q)

# ---------------------------------------------------------------------------
# Design constants - measured off the mockup, 1280x960 canvas
# ---------------------------------------------------------------------------

CANVAS = QSize(1280, 960)

GEO = {
    # Decorations, drawn behind every control.
    "header":       QRect(268,  70,  750, 112),   # rat + footprints
    "mouse_top":    QRect(1250,  0,  195, 179),
    "mouse_bottom": QRect(-140,   840,  215, 154),

    "title":        QRect(168,  38,  812, 104),   # text, drawn over "header"
    "bezel":        QRect(168, 148,  812, 556),
    "video_left":   QRect(192, 172,  376, 506),
    "video_right":  QRect(576, 172,  380, 506),

    # Vertical lamp slider: big sun (bright) on top, small sun at the bottom.
    "sun_max":      QRect(58,  194,   48,  55),
    "slider":       QRect(30,  254,  104, 300),
    "sun_min":      QRect(66,  560,   37,  42),

    # Boxes match each PNG's own aspect ratio, centred on the mockup position,
    # so fit() has no slack to letterbox away.
    "arrow_up":     QRect(1040, 385, 185, 207),
    "arrow_down":   QRect(1036, 618, 192, 214),

    # Lining actuator.
    "can":          QRect(425, 740,  163, 165),

    # Automation column down the left, under the lamp slider.
    "start":        QRect(38,  607,   77,  76),
    "stop":         QRect(38,  697,   76,  77),
    "home":         QRect(38,  785,   77,  77),
    "folder":       QRect(215, 792,  107,  75),

    # Odometer. The number is drawn into the pale window in the artwork.
    "distance":     QRect(1022, 165, 236, 196),

    # Hook group - three servo positions, left to right. Keys match the names
    # in HOOK_POSITIONS; swap these rects if you reorder that list.
    "pos_hook":     QRect(676, 748,   60, 138),
    "pos_assembly": QRect(744, 784,  134,  59),
    "pos_deploy":   QRect(885, 750,   59, 134),

    # Kept even when SHOW_STATUS is False - the widget is just hidden, and
    # _build_status still needs somewhere to put it.
    "status":       QRect(0,   906, 1280,  48),   # clears the spool at y=905
}

TITLE_TEXT = "repair rat #64"

# ---------------------------------------------------------------------------
# THE HOOK POSITIONS - single source of truth.
#
# Listed LEFT TO RIGHT as they appear on screen. Each entry is:
#   (asset/GEO name, serial char, servo angle, label shown in the status bar)
#
# The angles come from src/main.cpp:
#     case 'H': hookServo.setAngle(180)
#     case 'A': hookServo.setAngle(90)
#     case 'U': hookServo.setAngle(0)
# and they line up with the artwork: upright at 180, side-on at 90, down at 0.
#
# To reorder the buttons on screen, reorder this list AND swap the matching
# rects in GEO. To rename one, change it here and rename its two asset files.
# ---------------------------------------------------------------------------
HOOK_POSITIONS = [
    ("pos_hook",     "H", 180, "hook"),
    ("pos_assembly", "A",  90, "assembly"),
    ("pos_deploy",   "U",   0, "deploy"),
]

HOOK_LABEL = {cmd: label for _, cmd, _, label in HOOK_POSITIONS}
HOOK_ANGLE = {cmd: ang for _, cmd, ang, _ in HOOK_POSITIONS}

# Where the servo actually sits at power-on. main.cpp line 57:
#     hookServo.begin(PIN_SERVO, 180)   ->  180 is 'H'
# Showing anything else would be a lie about the hardware before you have
# touched a button.
HOOK_BOOT = "H"

# ---------------------------------------------------------------------------
# THE TWO INDEPENDENT TOGGLES
#
# The can and the spool are separate mechanisms on separate hardware, so each
# is its own two-state button rather than a pair that fight over one motor.
#
# LINING is the actuator (a motor). 'D' extends it, 'R' pulls it back, and
# Actuator::update() cuts power after ACT_RUN_S either way.
#
# SPOOL is a servo of its own on PIN_SPOOL (main.cpp lines 152-153):
#     case 'T': spoolServo.setAngle(180)
#     case 'Y': spoolServo.setAngle(90)
# and main.cpp line 62 starts it with spoolServo.begin(PIN_SPOOL, 90), so 90
# ('Y') is the power-on position and therefore the "off" artwork.
#
# Each entry is (off command, on command, off label, on label, starts on).
# ---------------------------------------------------------------------------
LINING = ("R", "D", "compressed", "deployed", False)
# The spool has no button any more, but the servo still exists on the robot,
# so T and Y stay on the keyboard. There is no artwork to show its state.
SPOOL = ("Y", "T", "string out", "wound in", False)

# ---------------------------------------------------------------------------
# THE ODOMETER
#
# sendTelemetry() in main.cpp emits {"pos_mm": steps / STEPS_PER_MM every
# TELEM_INTERVAL_MS. The panel counts nothing itself; it divides by 10 for cm,
# which is the same number control_panel_stereo2 shows.
#
# 'Z' runs stepper.zero() and imu.zero(). Clicking the readout sends it.
# The reading is SIGNED - reversing counts down. It is a position along the
# pipe, not a total-distance trip meter.
# ---------------------------------------------------------------------------
ODO_CMD_ZERO = "Z"
ODO_FIELD = "pos_mm"
ODO_FIELD_FALLBACK = "steps"
ODO_SCREEN = (0.076, 0.097, 0.924, 0.724)     # window inside the artwork
# Only for firmware that sends "steps" but not "pos_mm". Must match main.cpp;
# test_panel.py checks that it does.
STEPS_PER_MM = 3.222
# Seconds before a reading blanks to "--". None keeps the last number on
# screen, matching control_panel_stereo2.
ODO_STALE_S = None

# ---------------------------------------------------------------------------
# THE LAMP
#
# There is no on/off button in the artwork, so this lives on the keyboard:
#     L        toggle on / off        ('L' = led.on(), 'O' = led.off())
#     = or +   one step brighter      sends '0'-'9'
#     - or _   one step dimmer
#
# LED::begin() (src/LED.cpp) sets _state = false and _bright = 255, so the
# lamp boots OFF with full brightness remembered - hence the slider starting
# at 9 while the status line reads "off".
#
# The catch: LED::setBrightness does `_state = (b > 0)`, so sending any digit
# also switches the lamp ON, and sending '0' switches it OFF. Moving the
# slider is therefore an on/off command as well as a level, and on_led has to
# mirror that or the panel's idea of the lamp goes stale.
# ---------------------------------------------------------------------------
LED_ON, LED_OFF = "L", "O"
LED_BOOT_ON = False        # LED::begin -> _state = false
LED_BOOT_LEVEL = 9         # LED::begin -> _bright = 255, which is '9'

# The diagnostic strip along the bottom. Off for the showcase, on when you are
# debugging - it reports link state, fps, every position, the lamp, the last
# character sent, crack count and distance, and the telemetry echo.
# Prefer flipping this to commenting the code out: closeEvent lives in the same
# block, and without it the drive motor keeps running after the window shuts.
SHOW_STATUS = False

C_BG        = QColor("#E4E2DD")
C_INK       = QColor("#1B4EA0")
C_BLUE      = QColor("#7BA3D8")   # video pane border

# Protocol
MAGIC = b"\xAA\x55"
ID_MAP = {0x00: "L", 0x4C: "L",     # 0x4C = 'L'
          0x01: "R", 0x52: "R"}     # 0x52 = 'R'
TELEM_ID = 0x02
MAX_FRAME = 200_000
SOI, EOI = b"\xFF\xD8", b"\xFF\xD9"

# Set True only if the hook physically fouls the wheels while it is holding
# the string loops. Blocks F/B whenever the hook is in the 'H' position.
INTERLOCK_HOOK_BLOCKS_DRIVE = False

# Must match Actuator::MAX_RUN_MS in "THIS ONE/include/Actuator.h" (10000 ms).
# The firmware is what actually stops the motor; this is only for the on-screen
# countdown. If you change one, change the other.
ACT_RUN_S = 10.0

_TELEM_RE = re.compile(r'"?(\w+)"?\s*[:=]\s*(-?\d+(?:\.\d+)?)')



# ---------------------------------------------------------------------------
# Artwork loading
# ---------------------------------------------------------------------------

# Everything else in ASSETS is required. These are the extra states: without
# them a button still works, it just falls back to a generated tint/fade.
OPTIONAL_ASSETS = {k for k in ASSETS if k.endswith(("_on", "_down"))}


def check_assets():
    """Report every missing file at once, before the window is built.

    There are no drawn placeholders any more, so a typo used to mean a blank
    button with a line buried in the console. Fail loudly and list all of them
    together instead of one per run.
    """
    problems = []
    for key, path in ASSETS.items():
        if not path:
            if key not in OPTIONAL_ASSETS:
                problems.append(f"  {key:18s} not set in ASSETS")
            continue
        full = asset_path(path)
        if not os.path.exists(full):
            problems.append(f"  {key:18s} file not found: {path}")
    return problems


def fit(pm: QPixmap, size: QSize) -> QPixmap:
    """Scale to fit `size` keeping the aspect ratio, centred on a transparent
    canvas of exactly `size`.

    GEO rects are therefore bounding boxes, not exact art dimensions: artwork
    is never stretched, so a rect that is slightly the wrong shape shows a
    little slack rather than a squashed picture.
    """
    canvas = QPixmap(size)
    canvas.fill(Qt.transparent)
    sc = pm.scaled(size, Qt.KeepAspectRatio, Qt.SmoothTransformation)
    p = QPainter(canvas)
    p.drawPixmap((size.width() - sc.width()) // 2,
                 (size.height() - sc.height()) // 2, sc)
    p.end()
    return canvas


def _as_float(v):
    """Telemetry values arrive as strings. Returns None rather than raising,
    so one malformed field cannot take the odometer down."""
    if v is None:
        return None
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def load(key: str, size: QSize) -> QPixmap:
    """Required artwork. check_assets() has already run, so this should not
    fail; if the file is corrupt rather than missing, say so and stop."""
    pm = QPixmap(asset_path(ASSETS[key]))
    if pm.isNull():
        raise SystemExit(f"[assets] {ASSETS[key]!r} could not be decoded "
                         f"(needed for {key!r}). Is it a valid PNG?")
    return pm.scaled(size, Qt.KeepAspectRatio, Qt.SmoothTransformation)


def load_opt(key: str, size: QSize):
    """Optional artwork - the pressed arrows and the lit group buttons.
    Returns None if not configured, and the caller generates a fallback."""
    path = ASSETS.get(key)
    if not path:
        return None
    pm = QPixmap(asset_path(path))
    if pm.isNull():
        print(f"[assets] could not decode {path!r} for {key!r}; "
              "falling back to a generated state")
        return None
    return pm.scaled(size, Qt.KeepAspectRatio, Qt.SmoothTransformation)


# ---------------------------------------------------------------------------
# Widgets
# ---------------------------------------------------------------------------

class ArtButton(QPushButton):
    """A button that is exactly its artwork.

    Clicks only land on non-transparent pixels, so the two touching arrows and
    the hook's empty notch behave the way they look. Because the firmware has
    no watchdog, leaving the button while held also fires `released` - a motor
    left running is worse than a jerky UI.

    rightClicked gives a second action per button without needing more art.
    """

    rightClicked = Signal()

    def __init__(self, pixmap: QPixmap, rect: QRect, parent=None,
                 down_pixmap: QPixmap = None):
        super().__init__(parent)
        fitted = fit(pixmap, rect.size())
        self._normal = fitted
        self._hover = self._tint(fitted, 1.10)
        self._down = (fit(down_pixmap, rect.size())
                      if down_pixmap is not None and not down_pixmap.isNull()
                      else self._tint(fitted, 0.88))
        self.setGeometry(rect)
        self.setIconSize(rect.size())
        self.setIcon(QIcon(self._normal))
        self.setCursor(Qt.PointingHandCursor)
        self.setFocusPolicy(Qt.NoFocus)
        self.setStyleSheet("border:none; background:transparent;")
        self._apply_mask(self._normal, self._down)

    def _apply_mask(self, *pixmaps):
        """Clip the widget to the artwork, using the UNION of every state.

        setMask clips all painting, not just clicks. Masking to one state
        would crop any other state that pokes outside it - the deployed can
        and the wound-in spool have different silhouettes from their off
        images, so their lit artwork would have been partly invisible.
        """
        union = QPixmap(self.size())
        union.fill(Qt.transparent)
        p = QPainter(union)
        for pm in pixmaps:
            if pm is not None and not pm.isNull():
                p.drawPixmap(0, 0, pm)
        p.end()
        mask = union.mask()
        if not mask.isNull():
            self.setMask(mask)

    @staticmethod
    def _tint(pm: QPixmap, factor: float) -> QPixmap:
        """Lighten or darken without touching transparent pixels.

        Done with one composited fill rather than a Python pixel loop - at
        these button sizes the loop version took seconds of startup time.
        SourceAtop keeps the destination's alpha, so the silhouette survives.
        """
        out = pm.copy()
        shade = 255 if factor >= 1 else 0
        alpha = min(255, int(abs(1.0 - factor) * 255))
        p = QPainter(out)
        p.setCompositionMode(QPainter.CompositionMode_SourceAtop)
        p.fillRect(out.rect(), QColor(shade, shade, shade, alpha))
        p.end()
        return out

    # ---- the four mouse handlers below only swap which picture is showing,
    # except leaveEvent, which also has a safety job. Qt gives a plain
    # QPushButton hover/pressed looks for free via its stylesheet; these
    # buttons are images, so the swap has to be done by hand.

    def enterEvent(self, e):
        """Cursor moved onto the button: show the slightly brighter version."""
        if not self.isDown():
            self.setIcon(QIcon(self._hover))
        super().enterEvent(e)

    def leaveEvent(self, e):
        """Cursor left the button. Two jobs:

        1. Go back to the normal picture.
        2. SAFETY. If the mouse slides off while the button is still held, a
           press-and-hold control would otherwise never see its release and
           the motor would keep running - the stepper has no timeout. Force
           the release here so 'S' is sent.
        """
        if self.isDown():
            self.setDown(False)
            self.released.emit()
        self.setIcon(QIcon(self._normal))
        super().leaveEvent(e)

    def mousePressEvent(self, e):
        """Button pushed down: show the pressed artwork.

        Right-click is diverted to rightClicked and deliberately does NOT go
        to super(), so it cannot start a press-and-hold the user can never
        end. Nothing uses right-click at the moment.
        """
        if e.button() == Qt.RightButton:
            self.rightClicked.emit()
            return
        self.setIcon(QIcon(self._down))
        super().mousePressEvent(e)

    def mouseReleaseEvent(self, e):
        """Button let go: back to hover if the cursor is still over it,
        otherwise the normal picture. super() is what emits `clicked`."""
        self.setIcon(QIcon(self._hover if self.underMouse() else self._normal))
        super().mouseReleaseEvent(e)


class SelectableButton(ArtButton):
    """An ArtButton that shows whether it is the active choice in a group.

    Selected draws the "_on" artwork if you supplied one, otherwise the base
    image at full strength. Unselected fades the image toward the background,
    which is what the pale blue shapes in the mockup are.
    """

    def __init__(self, pixmap: QPixmap, rect: QRect, parent=None,
                 on_pixmap: QPixmap = None, selected=False,
                 dim_selected=False):
        super().__init__(pixmap, rect, parent)
        fitted = fit(pixmap, rect.size())
        has_on = on_pixmap is not None and not on_pixmap.isNull()

        if dim_selected and not has_on:
            # Inverted: the SELECTED one is greyed out, the other stays full
            # colour. Used for start/stop, where the greyed button reads as
            # "this is the mode you are in" rather than "this is disabled".
            self._sel_pm = self._fade(fitted, 0.45)
            self._unsel_pm = fitted
            self._selected = None
            self._apply_mask(fitted)
            self.set_selected(selected)
            return

        self._sel_pm = fit(on_pixmap, rect.size()) if has_on else fitted
        # With separate on/off artwork the off state is used as drawn. Without
        # it, fade the base image toward the background so the two states still
        # read differently.
        self._unsel_pm = fitted if has_on else self._fade(fitted, 0.45)
        self._selected = None
        # Both states must be clickable and fully visible, so widen the mask
        # the base class built from the off artwork alone.
        self._apply_mask(self._sel_pm, self._unsel_pm)
        self.set_selected(selected)

    @staticmethod
    def _fade(pm: QPixmap, amount: float) -> QPixmap:
        """Blend toward the panel background, keeping the alpha shape intact."""
        out = pm.copy()
        p = QPainter(out)
        p.setCompositionMode(QPainter.CompositionMode_SourceAtop)
        p.fillRect(out.rect(), QColor(C_BG.red(), C_BG.green(), C_BG.blue(),
                                      int(amount * 255)))
        p.end()
        return out

    def set_selected(self, on: bool):
        """THIS is what swaps the artwork between the on and off images.

        Called only by ButtonGroup - never call it directly, or the group's
        idea of what is selected and the picture on screen will disagree.
        """
        if on == self._selected:
            return
        self._selected = on
        self._normal = self._sel_pm if on else self._unsel_pm
        self._hover = self._tint(self._normal, 1.12)
        self._down = self._tint(self._normal, 0.86)
        self.setIcon(QIcon(self._normal))

    def is_selected(self) -> bool:
        return bool(self._selected)


class _HeadlessToggleButton:
    """Stand-in for a SelectableButton that has no artwork on the panel.

    The spool servo still exists on the robot but lost its buttons, so its
    Toggle keeps working from the keyboard with one of these in place of a
    widget. Everything is a no-op except remembering the state.
    """

    def __init__(self):
        self.sel = None
        self.clicked = _NullSignal()

    def set_selected(self, on):
        self.sel = on

    def is_selected(self):
        return bool(self.sel)

    def setToolTip(self, _t):
        pass


class _NullSignal:
    def connect(self, _fn):
        pass


class ScreenshotGallery(QDialog):
    """The FOLDER button's window: thumbnails of this run's flagged cracks.

    Reads the folder fresh each time it opens, so it picks up anything saved
    since last look. Click a thumbnail to see it full size.
    """

    THUMB = QSize(220, 165)

    def __init__(self, folder, parent=None):
        super().__init__(parent)
        self.folder = folder
        self.setWindowTitle("Flagged cracks")
        self.resize(760, 560)
        self.setStyleSheet(f"background:{C_BG.name()};")

        outer = QVBoxLayout(self)
        self.caption = QLabel(self)
        self.caption.setStyleSheet(
            f"color:{C_INK.name()}; font-family:Arial; font-size:13px;")
        outer.addWidget(self.caption)

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("border:none;")
        inner = QWidget()
        self.grid = QGridLayout(inner)
        self.grid.setSpacing(10)
        scroll.setWidget(inner)
        outer.addWidget(scroll, 1)

        self.reload()

    def reload(self):
        while self.grid.count():
            item = self.grid.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        shots = []
        if self.folder and os.path.isdir(self.folder):
            shots = sorted(
                os.path.join(self.folder, f) for f in os.listdir(self.folder)
                if f.lower().endswith((".png", ".jpg", ".jpeg")))

        if not shots:
            self.caption.setText(
                f"No screenshots yet.\n{self.folder or 'no folder yet'}\n"
                "Cracks are only captured while a scan is running, and only "
                f"when confidence is between {scv_conf_min()} and "
                f"{scv_conf_max()}.")
            return

        self.caption.setText(f"{len(shots)} flagged this run  ·  {self.folder}")
        for i, path in enumerate(shots):
            pm = QPixmap(path)
            if pm.isNull():
                continue
            cell = QLabel(self)
            cell.setPixmap(pm.scaled(self.THUMB, Qt.KeepAspectRatio,
                                     Qt.SmoothTransformation))
            cell.setToolTip(os.path.basename(path))
            cell.setCursor(Qt.PointingHandCursor)
            cell.mousePressEvent = lambda _e, p=path: self._open_full(p)
            name = QLabel(os.path.basename(path), self)
            name.setStyleSheet(
                f"color:{C_INK.name()}; font-family:Menlo,monospace;"
                "font-size:10px;")
            name.setWordWrap(True)
            name.setFixedWidth(self.THUMB.width())
            row, col = divmod(i, 3)
            self.grid.addWidget(cell, row * 2, col)
            self.grid.addWidget(name, row * 2 + 1, col)

    def _open_full(self, path):
        dlg = QDialog(self)
        dlg.setWindowTitle(os.path.basename(path))
        dlg.setStyleSheet("background:#111;")
        lay = QVBoxLayout(dlg)
        lbl = QLabel(dlg)
        pm = QPixmap(path)
        lbl.setPixmap(pm.scaled(QSize(900, 700), Qt.KeepAspectRatio,
                                Qt.SmoothTransformation))
        lay.addWidget(lbl)
        dlg.exec()


def scv_conf_min():
    return auto.SCREENSHOT_CONF_MIN if HAVE_AUTO else "?"


def scv_conf_max():
    return auto.SCREENSHOT_CONF_MAX if HAVE_AUTO else "?"


class Toggle:
    """ONE button that flips between two states, each with its own command.

    This is the other kind of control. ButtonGroup below is "pick one of
    several, the rest go dark" - that is the hook, where three positions share
    one servo. A Toggle is a single button that alternates: click it and it
    sends the on command and shows the lit artwork, click again and it sends
    the off command and goes back. The picture is always the last state you
    commanded, so the button doubles as the readout.

    The can and the spool are one of these each, because they are separate
    mechanisms and neither constrains the other.
    """

    def __init__(self, button: SelectableButton, spec, on_change):
        # spec is (off_cmd, on_cmd, off_label, on_label, starts_on)
        self.off_cmd, self.on_cmd, self.off_label, self.on_label, start = spec
        self.button = button
        self.on_change = on_change
        self._on = bool(start)
        button.set_selected(self._on)
        # Same Qt trap as ButtonGroup.add: clicked carries a bool, so swallow
        # any argument rather than let it become a parameter we care about.
        button.clicked.connect(lambda *_a: self.flip())

    def flip(self):
        self.set(not self._on)

    def set(self, on: bool):
        """Command a state. Sends even if already there, so a click always
        re-commands the hardware rather than silently doing nothing."""
        self._on = bool(on)
        self.button.set_selected(self._on)
        self.on_change(self.on_cmd if self._on else self.off_cmd, self._on)

    def is_on(self) -> bool:
        return self._on

    def label(self) -> str:
        return self.on_label if self._on else self.off_label


class ButtonGroup:
    """Mutually exclusive set of SelectableButtons - "radio buttons".

    This is the thing that makes "only one of the three can be on". Clicking a
    member selects it, switches every other member off, and fires its command.
    Each button still has just two states, on and off; the group is what
    guarantees exactly one of them is on.

    Clicking the already-selected member re-fires its command, because for the
    lining group that means "run it again" rather than "do nothing".

    Membership is declared by calling add() once per button - see
    Panel._build_controls, where self.lining gets D and R, and self.hook gets
    one entry per row of HOOK_POSITIONS.
    """

    def __init__(self, on_choose):
        self.on_choose = on_choose
        self.items = {}          # key -> SelectableButton

    def add(self, key, button: SelectableButton):
        """Put a button in this group. Called once per button; the group then
        owns its selected/unselected state for the rest of the session."""
        self.items[key] = button
        # The *_a is load-bearing. QPushButton.clicked carries a `checked`
        # bool, and PySide6 passes it to any slot willing to take an argument.
        # With `lambda k=key:` that bool lands in k, so every click called
        # choose(False): no key matched, every button switched off, and the
        # command was sent as a bool. Swallow the Qt argument instead.
        button.clicked.connect(lambda *_a, k=key: self.choose(k))

    def choose(self, key):
        for k, b in self.items.items():
            b.set_selected(k == key)
        self.on_choose(key)

    def set_selected(self, key):
        """Reflect state without firing the command."""
        for k, b in self.items.items():
            b.set_selected(k == key)

    def selected(self):
        for k, b in self.items.items():
            if b.is_selected():
                return k
        return None


class ImageSlider(QSlider):
    """Vertical slider drawn from a groove pixmap and a handle pixmap.

    Top is maximum, matching the big sun above and the small sun below.
    Click anywhere to jump, drag to scrub.
    """

    def __init__(self, handle: QPixmap, groove: QPixmap, rect: QRect,
                 lo=0, hi=9, parent=None):
        super().__init__(Qt.Vertical, parent)
        self._handle = handle
        self._groove = groove
        self.setGeometry(rect)
        self.setRange(lo, hi)
        self.setValue(hi)
        self.setCursor(Qt.PointingHandCursor)
        self.setFocusPolicy(Qt.NoFocus)

    def _travel(self) -> int:
        return max(1, self.height() - self._handle.height())

    def _frac(self) -> float:
        span = max(1, self.maximum() - self.minimum())
        return (self.value() - self.minimum()) / span

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        gy = self._handle.height() // 2
        gx = (self.width() - self._groove.width()) // 2
        p.drawPixmap(gx, gy, self._groove.scaled(
            self._groove.width(), max(1, self.height() - self._handle.height()),
            Qt.IgnoreAspectRatio, Qt.SmoothTransformation))
        # frac 1.0 is the TOP of the widget, so invert for y.
        hy = int((1.0 - self._frac()) * self._travel())
        hx = (self.width() - self._handle.width()) // 2
        p.drawPixmap(hx, hy, self._handle)
        p.end()

    def _set_from_y(self, y: float):
        frac = 1.0 - (y - self._handle.height() / 2) / self._travel()
        frac = min(1.0, max(0.0, frac))
        self.setValue(round(self.minimum() + frac * (self.maximum() - self.minimum())))

    def mousePressEvent(self, e):
        self._set_from_y(e.position().y())

    def mouseMoveEvent(self, e):
        self._set_from_y(e.position().y())


def bgr_to_qimage(bgr) -> QImage:
    """OpenCV BGR ndarray -> QImage.

    The .copy() is not optional: QImage wraps the numpy buffer without owning
    it, so once the array is garbage collected the pixmap shows torn memory.
    """
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    h, w = rgb.shape[:2]
    rgb = np.ascontiguousarray(rgb)
    return QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy()


class VideoPane(QLabel):
    def __init__(self, rect: QRect, active: bool, parent=None, border_px=2):
        super().__init__(parent)
        self._bw = max(1, border_px)
        self.setGeometry(rect)
        self.setAlignment(Qt.AlignCenter)
        self.set_active(active)

    def set_active(self, active: bool):
        border = f"{self._bw}px solid {C_BLUE.name()}" if active else "none"
        self.setStyleSheet(f"background:#000000; border:{border};")

    def show_frame(self, img: QImage):
        self.setPixmap(QPixmap.fromImage(img).scaled(
            self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))


# ---------------------------------------------------------------------------
# Serial layer - one owner of the port, nothing else touches it
# ---------------------------------------------------------------------------

def autodetect_port():
    if not HAVE_SERIAL:
        return None
    for p in list_ports.comports():
        d = (p.device + " " + (p.description or "")).lower()
        if any(k in d for k in ("usbserial", "usbmodem", "ttyusb", "ttyacm", "wchusb", "slab")):
            return p.device
    return None


class SerialLink(QObject):
    """Reader thread parses the tagged stream and emits Qt signals (delivered
    on the GUI thread). Writer thread drains a queue so no button handler can
    ever block on serial IO."""

    # Raw JPEG bytes, not a QImage: the panel decodes once with OpenCV so the
    # same ndarray feeds both the CV pipeline and the display.
    frame_ready = Signal(str, bytes)      # "L"/"R", jpeg
    telemetry = Signal(str)
    link_state = Signal(str)

    def __init__(self, port, baud=921600, parent=None):   # matches control_panel_stereo2
        super().__init__(parent)
        self.port_name = port
        self.baud = baud
        self.ser = None
        self._tx = queue.Queue()
        self._stop = threading.Event()
        self._buf = bytearray()
        self._textbuf = bytearray()      # loose telemetry text between frames
        self._last_t = {"L": time.time(), "R": time.time()}
        self.fps = {"L": 0.0, "R": 0.0}

    def start(self):
        if self.port_name and HAVE_SERIAL:
            try:
                self.ser = serial.Serial(self.port_name, self.baud, timeout=0.05)
                self.link_state.emit(f"LINK {self.port_name} @{self.baud}")
            except Exception as exc:
                self.link_state.emit(f"SIM (open failed: {exc})")
        else:
            why = "pyserial missing" if not HAVE_SERIAL else "no port found"
            self.link_state.emit(f"SIM ({why})")
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._writer, daemon=True).start()

    def stop(self):
        for c in "SX":                  # stop drive, stop actuator
            self.send(c)
        time.sleep(0.15)
        self._stop.set()
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass

    # -- tx ----------------------------------------------------------------
    def send(self, ch: str):
        self._tx.put(ch)

    def _writer(self):
        while not self._stop.is_set():
            try:
                ch = self._tx.get(timeout=0.1)
            except queue.Empty:
                continue
            if self.ser:
                try:
                    self.ser.write(ch.encode())
                except Exception as exc:
                    self.link_state.emit(f"TX ERROR {exc}")

    # -- rx ----------------------------------------------------------------
    def _reader(self):
        if self.ser:
            while not self._stop.is_set():
                try:
                    chunk = self.ser.read(8192)
                except Exception as exc:
                    self.link_state.emit(f"RX ERROR {exc}")
                    return
                if chunk:
                    self._buf += chunk
                    self._parse()
        else:
            self._sim()

    def _stash_text(self, chunk: bytes):
        """Bytes that are not part of any frame.

        sendTelemetry() in main.cpp uses plain Serial.print - it is NOT wrapped
        in the AA 55 frame - so its text arrives interleaved between frames.
        Discarding non-magic bytes, which is the obvious thing to do, drops
        every telemetry line on the floor and the odometer never updates.
        """
        self._textbuf += chunk
        while True:
            nl = self._textbuf.find(b"\n")
            if nl < 0:
                break
            line = bytes(self._textbuf[:nl])
            del self._textbuf[:nl + 1]
            text = line.decode("ascii", "replace").strip()
            if text:
                self.telemetry.emit(text)
        if len(self._textbuf) > 4096:      # binary junk, no newline coming
            del self._textbuf[:-256]

    def _parse(self):
        while True:
            i = self._buf.find(MAGIC)
            if i < 0:
                # Hold back the last byte ONLY if it could be the first half
                # of a magic split across two reads. Parking it
                # unconditionally strands a trailing newline, and the
                # telemetry line it terminates is never emitted.
                keep = 1 if self._buf.endswith(MAGIC[:1]) else 0
                cut = len(self._buf) - keep
                if cut > 0:
                    self._stash_text(bytes(self._buf[:cut]))
                    del self._buf[:cut]
                return
            if i:
                self._stash_text(bytes(self._buf[:i]))
                del self._buf[:i]
            if len(self._buf) < 7:
                return
            fid = self._buf[2]
            (length,) = struct.unpack_from("<I", self._buf, 3)
            if not (0 < length <= MAX_FRAME):
                del self._buf[:2]               # desync -> resync
                continue
            if len(self._buf) < 7 + length:
                return
            payload = bytes(self._buf[7:7 + length])
            del self._buf[:7 + length]

            if fid == TELEM_ID:
                self.telemetry.emit(payload.decode("ascii", "replace").strip())
                continue
            cam = ID_MAP.get(fid)
            if cam is None:
                continue
            # The sensor pads garbage after the image; trim to the end marker.
            if payload[:2] != SOI:
                continue
            end = payload.rfind(EOI)
            if end < 0:
                continue
            self._tick(cam)
            self.frame_ready.emit(cam, payload[:end + 2])

    def _tick(self, cam):
        now = time.time()
        dt = now - self._last_t[cam]
        self._last_t[cam] = now
        if dt > 0:
            self.fps[cam] = 0.85 * self.fps[cam] + 0.15 * (1.0 / dt)

    def _sim(self):
        """Animated test pattern, encoded to JPEG so it goes through exactly
        the same decode path as a real frame."""
        w, h = 320, 240
        t0 = time.time()
        while not self._stop.is_set():
            t = time.time() - t0
            for cam, phase in (("L", 0.0), ("R", 0.35)):
                img = QImage(w, h, QImage.Format_RGB888)
                img.fill(QColor(14, 14, 18))
                p = QPainter(img)
                p.setRenderHint(QPainter.Antialiasing, True)
                p.setPen(QPen(QColor(40, 44, 52), 1))
                for g in range(0, w, 32):
                    p.drawLine(g, 0, g, h)
                for g in range(0, h, 32):
                    p.drawLine(0, g, w, g)
                cx = w / 2 + math.sin(t * 1.1 + phase) * w * 0.26
                cy = h / 2 + math.cos(t * 0.7) * h * 0.22
                p.setPen(QPen(QColor("#00E676"), 2))
                p.setBrush(Qt.NoBrush)
                p.drawRect(int(cx - 34), int(cy - 22), 68, 44)
                p.setFont(QFont("Arial", 9))
                p.drawText(int(cx - 34), int(cy - 27), "sim target")
                p.setPen(QPen(QColor("#8899AA"), 1))
                p.drawText(8, 16, f"SIMULATED {cam}")
                p.end()

                buf = QBuffer()
                buf.open(QBuffer.WriteOnly)
                img.save(buf, "JPG", 80)
                self._tick(cam)
                self.frame_ready.emit(cam, bytes(buf.data()))
            time.sleep(1 / 20)


# ---------------------------------------------------------------------------
# Panel
# ---------------------------------------------------------------------------

class Panel(QWidget):

    def __init__(self, link: SerialLink, use_cv=True, conf=0.4,
                 win_size: QSize = None):
        super().__init__()
        self.link = link
        self._use_cv = use_cv and HAVE_CV
        self._conf = conf
        self.setWindowTitle("rat #88")

        # The mockup is authored at 1280x960. Rather than reflow the layout for
        # each screen, the whole design is scaled by a single factor and
        # centred, so GEO stays in design coordinates and the proportions the
        # artwork was drawn for are preserved. Uniform scale, never stretched.
        win = win_size or CANVAS
        self.setFixedSize(win)
        self.S = min(win.width() / CANVAS.width(), win.height() / CANVAS.height())
        self.ox = int((win.width() - CANVAS.width() * self.S) / 2)
        self.oy = int((win.height() - CANVAS.height() * self.S) / 2)
        self.win = win
        self.setFocusPolicy(Qt.StrongFocus)      # needed for arrow-key driving

        self.hook_pos = HOOK_BOOT    # HookServo::begin parks at 180 == 'H'
        self.act_until = 0.0
        self.act_dir = ""
        # LED::begin() leaves the lamp off with full brightness remembered, so
        # the slider sits at 9 while the lamp reads "off". Both are true.
        self.led_on = LED_BOOT_ON
        self._led_last = None
        self._gallery = None

        # The autonomous sequence. None if auto_sequence.py is missing, in
        # which case the panel stays a manual-control panel.
        self.auto = None
        if HAVE_AUTO:
            self.auto = auto.AutoSequence(
                send=self.send,
                get_pos_mm=self.current_pos_mm,
                on_status=self._set_auto_status)
        self.auto_status = "idle" if HAVE_AUTO else "automation unavailable"
        self._last_cmd = "-"
        self._telem = {}
        self._telem_seen = 0.0
        self.link_text = "starting"
        self._det_count = 0
        self._nearest = None
        self._dets = []

        # CV is optional at every level: missing opencv, missing ultralytics,
        # missing CV.pt and missing stereo_calib.npz each degrade separately
        # rather than stopping the panel from opening.
        self.cv = None
        self.cv_note = CV_IMPORT_ERROR or ""
        if self._use_cv:
            self.cv = scv.CVWorker(conf=self._conf)
            self.cv.start()
            notes = []
            if self.cv.cv_error:
                notes.append(self.cv.cv_error)
            if not self.cv.calib.ok:
                notes.append(self.cv.calib.error or "no calibration")
            self.cv_note = "  |  ".join(notes)

        self._build_background()
        self._build_video()
        self._build_controls()
        self._build_status()

        link.frame_ready.connect(self.on_frame)
        link.telemetry.connect(self.on_telemetry)
        link.link_state.connect(self._set_link_text)

        self.ticker = QTimer(self)
        self.ticker.timeout.connect(self.refresh_status)
        # 50ms, close to the 40ms tick the sequence was tuned against in
        # control_panel_stereo2 — AUTO_PAUSE_SETTLE_TICKS and
        # AUTO_PAUSE_CONFIRM_TICKS count ticks, so the rate is behaviour.
        self.ticker.start(50)

    # -- geometry ----------------------------------------------------------
    def g(self, key: str) -> QRect:
        """Design-space rect -> on-screen rect."""
        r = GEO[key]
        S = self.S
        return QRect(int(r.x() * S) + self.ox, int(r.y() * S) + self.oy,
                     int(r.width() * S), int(r.height() * S))

    def px(self, n: float) -> int:
        """Design-space length (font size, pixmap edge) -> on-screen pixels."""
        return max(1, int(round(n * self.S)))

    # -- construction ------------------------------------------------------
    def _build_background(self):
        pm = load("background", CANVAS)
        self.bg = QLabel(self)
        self.bg.setGeometry(QRect(QPoint(0, 0), self.win))
        # "Cover", not "stretch": fill the screen and crop the overflow so the
        # artwork keeps its proportions on a wider-than-4:3 laptop panel.
        self.bg.setPixmap(pm.scaled(self.win, Qt.KeepAspectRatioByExpanding,
                                    Qt.SmoothTransformation))
        self.bg.setAlignment(Qt.AlignCenter)

        # Decorations sit above the plate but below every control. Created
        # first so Qt's sibling stacking puts the buttons on top of them.
        self.decor = []
        for key in ("header", "mouse_top", "mouse_bottom"):
            rect = self.g(key)
            lbl = QLabel(self)
            lbl.setGeometry(rect)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setScaledContents(False)
            lbl.setPixmap(fit(load(key, rect.size()), rect.size()))
            self.decor.append(lbl)

        self.bezel = QLabel(self)
        self.bezel.setGeometry(self.g("bezel"))
        self.bezel.setPixmap(load("bezel", self.g("bezel").size()).scaled(
            self.g("bezel").size(), Qt.IgnoreAspectRatio,
            Qt.SmoothTransformation))

        # Drawn over the header art, which is only the rat and its footprints.
        self.title = QLabel(TITLE_TEXT, self)
        self.title.setGeometry(self.g("title"))
        self.title.setAlignment(Qt.AlignCenter)
        self.title.setStyleSheet(
            f"color:{C_INK.name()}; font-family:Arial;"
            f"font-size:{self.px(64)}px; font-weight:800; background:transparent;")

    def _build_video(self):
        bw = self.px(2)
        self.panes = {
            "L": VideoPane(self.g("video_left"), True, self, bw),
            "R": VideoPane(self.g("video_right"), False, self, bw),
        }

    def _build_controls(self):
        G = self.g

        # ---- drive: press and hold, because the stepper has no timeout ----
        up = load("arrow_up", G("arrow_up").size())
        dn = load("arrow_down", G("arrow_down").size())

        up_d = load_opt("arrow_up_down", G("arrow_up").size())
        dn_d = load_opt("arrow_down_down", G("arrow_down").size())

        self.btn_fwd = ArtButton(up, G("arrow_up"), self, up_d)
        self.btn_fwd.setToolTip("Drive forward — hold, or Up arrow key")
        self.btn_fwd.pressed.connect(lambda: self.drive("F"))
        self.btn_fwd.released.connect(lambda: self.send("S"))

        self.btn_back = ArtButton(dn, G("arrow_down"), self, dn_d)
        self.btn_back.setToolTip("Drive back — hold, or Down arrow key")
        self.btn_back.pressed.connect(lambda: self.drive("B"))
        self.btn_back.released.connect(lambda: self.send("S"))

        # ---- two independent toggles, one per mechanism -------------------
        # Separate hardware, so neither constrains the other: the lining is
        # the actuator, the spool is its own servo on PIN_SPOOL.
        can_btn = SelectableButton(load("can", G("can").size()), G("can"), self,
                                   load_opt("can_on", G("can").size()))
        can_btn.setToolTip(
            f"Lining actuator — click to deploy ('{LINING[1]}'), "
            f"click again to retract ('{LINING[0]}').\n"
            f"Firmware cuts the motor after {ACT_RUN_S:.0f}s either way.\n"
            "Space stops it immediately.")
        self.lining = Toggle(can_btn, LINING, self.on_lining)

        # The spool lost its artwork but not its servo, so it keeps a Toggle
        # with a stand-in button. T and Y still work; there is just nothing on
        # screen showing which way it is.
        self.spool = Toggle(_HeadlessToggleButton(), SPOOL, self.on_spool)

        # ---- automation: start and stop are one either/or pair -------------
        # Exactly one is lit at a time, and the lit one is the greyed one, so
        # the panel always shows whether the robot is running itself or
        # sitting under manual control. STOP starts out selected because
        # nothing is running yet.
        self.run = ButtonGroup(self.on_run)

        self.btn_start = SelectableButton(
            load("start", G("start").size()), G("start"), self,
            selected=False, dim_selected=True)
        self.btn_start.setToolTip("Start the automatic scan.\n"
                                  "Drives forward, stops at anything that "
                                  "looks like a crack, deploys a lining.")
        self.run.add("start", self.btn_start)

        self.btn_stop = SelectableButton(
            load("stop", G("stop").size()), G("stop"), self,
            selected=True, dim_selected=True)
        self.btn_stop.setToolTip("Stop everything — drive, actuator and any "
                                 "running sequence.\nSame as the space bar.")
        self.run.add("stop", self.btn_stop)

        self.btn_home = ArtButton(load("home", G("home").size()),
                                  G("home"), self)
        self.btn_home.setToolTip("Drive back to position 0, then zero the "
                                 "odometer.\nNeeds telemetry; refuses while a "
                                 "sequence is running.")
        self.btn_home.clicked.connect(lambda *_a: self.go_home())

        self.btn_folder = ArtButton(load("folder", G("folder").size()),
                                    G("folder"), self)
        self.btn_folder.setToolTip("Screenshots of ambiguous cracks from this "
                                   "run.")
        self.btn_folder.clicked.connect(lambda *_a: self.show_gallery())

        # ---- odometer: a readout that is also its own reset button --------
        rect = G("distance")
        self.btn_odo = ArtButton(load("distance", rect.size()), rect, self)
        self.btn_odo.setToolTip("Distance along the pipe.\n"
                                "Click to zero it (sends 'Z').")
        self.btn_odo.clicked.connect(lambda *_a: self.zero_odometer())

        # The number goes in the pale window inside the artwork. fit() centres
        # the art in its box, so find where the art actually landed first.
        art = load("distance", rect.size())
        ax = rect.x() + (rect.width() - art.width()) // 2
        ay = rect.y() + (rect.height() - art.height()) // 2
        fx0, fy0, fx1, fy1 = ODO_SCREEN
        win = QRect(ax + int(fx0 * art.width()), ay + int(fy0 * art.height()),
                    int((fx1 - fx0) * art.width()),
                    int((fy1 - fy0) * art.height()))
        self.odo = QLabel("--", self)
        self.odo.setGeometry(win)
        self.odo.setAlignment(Qt.AlignCenter)
        self.odo.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.odo.setStyleSheet(
            f"color:{C_INK.name()}; font-family:Menlo,Consolas,monospace;"
            f"font-size:{max(12, int(win.height() * 0.5))}px; font-weight:700;"
            "background:transparent;")

        # ---- hook group: exactly one position lit at a time ---------------
        # Built straight from HOOK_POSITIONS so the buttons, the commands, the
        # status labels and the keyboard shortcuts can never drift apart.
        self.hook = ButtonGroup(self.on_hook)
        for name, cmd, angle, label in HOOK_POSITIONS:
            rect = G(name)
            btn = SelectableButton(load(name, rect.size()), rect, self,
                                   load_opt(name + "_on", rect.size()))
            btn.setToolTip(f"{label}  ({cmd}, {angle}°)\n"
                           f"Keyboard: {cmd}")
            self.hook.add(cmd, btn)
        # Reflect the real boot position without sending a command.
        self.hook.set_selected(self.hook_pos)

        # ---- lamp: vertical, bright at the top ----------------------------
        self.sun_max = QLabel(self)
        self.sun_max.setGeometry(G("sun_max"))
        self.sun_max.setPixmap(load("sun_max", G("sun_max").size()))
        self.sun_min = QLabel(self)
        self.sun_min.setGeometry(G("sun_min"))
        self.sun_min.setPixmap(load("sun_min", G("sun_min").size()))

        handle = load("cheese", QSize(self.px(103), self.px(70)))
        groove = load("groove", QSize(self.px(104), self.px(330)))
        # Firmware maps the characters '0'-'9' onto 0-255, so the slider is 0-9
        # and one character carries the level.
        self.slider = ImageSlider(handle, groove, G("slider"), 0, LED_BOOT_LEVEL,
                                  self)
        self.slider.setToolTip("Lamp brightness (0 bottom - 9 top)")
        self.slider.valueChanged.connect(self.on_led)

    def _build_status(self):
        """The diagnostic strip along the bottom. SHOW_STATUS turns it off for
        the showcase; everything behind it keeps running either way."""
        self.status = QLabel(self)
        self.status.setGeometry(self.g("status"))
        self.status.setAlignment(Qt.AlignCenter)
        self.status.setStyleSheet(
            f"color:{C_INK.name()}; font-family:Menlo,Consolas,monospace;"
            f"font-size:{self.px(11)}px; background:transparent;")
        self.status.setVisible(SHOW_STATUS)

    # -- commands ----------------------------------------------------------
    def send(self, ch: str):
        self.link.send(ch)
        self._last_cmd = ch

    def drive(self, ch: str):
        if INTERLOCK_HOOK_BLOCKS_DRIVE and self.hook_pos == "H":
            self._last_cmd = "blocked: hook out"
            return
        self.send(ch)

    # Both handlers are called BY the Toggle, never directly. The Toggle has
    # already flipped its own state and swapped its artwork; all these do is
    # put the character on the wire and update the status line.
    #   ch  - the command character for the state just entered
    #   on  - True if that is the "on" state, for wording the status text

    def on_lining(self, ch: str, on: bool):
        """Lining actuator: 'D' deploys, 'R' retracts.

        This is a MOTOR, so the move takes real time. act_until is a countdown
        display only - it does not stop anything. Actuator::update() on the
        ESP32 is what cuts the power, after MAX_RUN_MS. We mirror the same
        duration here purely so the status line can show how long is left.
        """
        self.send(ch)
        self.act_until = time.time() + ACT_RUN_S
        self.act_dir = "deploying" if on else "retracting"

    def on_spool(self, ch: str, on: bool):
        """Spool servo: 'T' winds in (180 deg), 'Y' pays out (90 deg).

        No countdown, because a servo is not a motor that runs. setAngle drives
        it to the angle and holds there, so there is nothing to time and
        nothing to stop.
        """
        self.send(ch)

    def on_hook(self, ch: str):
        """Servo positions: A assemble, H hold the loops, U release."""
        self.send(ch)
        self.hook_pos = ch

    def set_hook(self, ch: str):
        """Move the hook and keep the button group in step."""
        self.hook.choose(ch)

    def on_led(self, v: int):
        # Only send on change; a drag otherwise floods the port and competes
        # with the video for bandwidth.
        if v != self._led_last:
            self._led_last = v
            self.send(str(v))
            # LED::setBrightness does _state = (b > 0), so the digit we just
            # sent has also switched the lamp on (or off, at 0). Mirror it.
            self.led_on = v > 0

    # ---- automation ------------------------------------------------------
    def current_pos_mm(self):
        """Odometer in mm, or None. This is the sequence's only position
        feedback, so returning None matters: it makes homing and repositioning
        fail safely instead of driving blind."""
        mm = _as_float(self._telem.get(ODO_FIELD))
        if mm is not None:
            return mm
        steps = _as_float(self._telem.get(ODO_FIELD_FALLBACK))
        return None if steps is None else steps / STEPS_PER_MM

    def on_run(self, key: str):
        """The start/stop pair. ButtonGroup has already lit one and dimmed the
        other; this decides what that means."""
        if key == "start":
            self.auto_start()
        else:
            self.stop_all()

    def sync_run_buttons(self):
        """Keep the pair honest.

        The sequence can end on its own - it finishes a lining, it reaches
        home, it loses telemetry - and the space bar and losing window focus
        also stop it. None of those go through the buttons, so without this
        the panel would sit there claiming to be running.

        set_selected, not choose: this reflects state, it must not re-issue
        the command.
        """
        want = "start" if (self.auto and self.auto.running) else "stop"
        if self.run.selected() != want:
            self.run.set_selected(want)

    def auto_start(self):
        if not self.auto:
            self._last_cmd = "automation unavailable"
            return
        if self.auto.running:
            self._last_cmd = "already running"
            return
        self.auto.start()

    def go_home(self):
        """HOME: drive back to 0, then zero the odometer once it arrives."""
        if not self.auto:
            self.zero_odometer()          # no sequence available: just zero
            return
        self.auto.return_home(zero_on_arrival=True)

    def show_gallery(self):
        folder = self.auto.inspection_dir if self.auto else None
        if self._gallery is None:
            self._gallery = ScreenshotGallery(folder, self)
        else:
            self._gallery.folder = folder
            self._gallery.reload()
        self._gallery.show()
        self._gallery.raise_()

    def zero_odometer(self):
        """Reset to 0. 'Z' calls stepper.zero() and imu.zero() on the ESP32.

        Blanks the display straight away rather than waiting for the next
        telemetry packet, because nothing acknowledges 'Z'.
        """
        self.send(ODO_CMD_ZERO)
        self._telem.pop(ODO_FIELD, None)
        self._telem.pop(ODO_FIELD_FALLBACK, None)
        self.odo.setText("0.0")

    def update_odometer(self):
        """Shows the same number as control_panel_stereo2: pos_mm / 10 in cm,
        falling back to the step count on firmware without pos_mm."""
        if (ODO_STALE_S is not None
                and (time.time() - self._telem_seen) > ODO_STALE_S):
            self.odo.setText("--")
            return
        mm = self.current_pos_mm()
        if mm is not None:
            self.odo.setText(f"{mm / 10.0:.1f}")
        elif not self._telem:
            self.odo.setText("--")

    def toggle_led(self):
        """L key. LED::on() restores the last brightness, so the slider
        position stays meaningful across an off/on cycle."""
        self.led_on = not self.led_on
        self.send(LED_ON if self.led_on else LED_OFF)

    def nudge_lamp(self, delta: int):
        """= and - keys. Moves the slider, which emits valueChanged and so
        goes through on_led - one path to the wire, same as the mouse."""
        lo, hi = self.slider.minimum(), self.slider.maximum()
        self.slider.setValue(min(hi, max(lo, self.slider.value() + delta)))

    def stop_all(self):
        """Panic: cancel any sequence, stop the drive, cut the lining actuator.

        The toggle is deliberately left where it is. Stopping mid-travel means
        the lining is somewhere between compressed and deployed, and flipping
        the button back would claim a position the hardware is not in. The
        status line says "stopped" instead. Servos are not touched - they are
        already parked and holding.
        """
        # Cancel the sequence FIRST. Otherwise its next tick would happily
        # re-send 'F' and the robot would carry on after the panic button.
        if self.auto:
            self.auto.stop()
        self.send("S")
        self.link.send("X")
        self.act_until = 0.0
        self.act_dir = "stopped"

    # -- events ------------------------------------------------------------
    def keyPressEvent(self, e):
        if e.isAutoRepeat():
            return
        k = e.key()
        if k == Qt.Key_Up:
            self.btn_fwd.setDown(True); self.drive("F")
        elif k == Qt.Key_Down:
            self.btn_back.setDown(True); self.drive("B")
        elif k == Qt.Key_Space:
            self.stop_all()
        elif e.text().upper() == ODO_CMD_ZERO:
            self.zero_odometer()      # same path as clicking the readout
        elif e.text().upper() in HOOK_LABEL:
            self.set_hook(e.text().upper())
        elif e.text().upper() == LINING[1]:      # D - deploy
            self.lining.set(True)
        elif e.text().upper() == LINING[0]:      # R - retract
            self.lining.set(False)
        elif e.text().upper() == SPOOL[1]:       # T - wind in
            self.spool.set(True)
        elif e.text().upper() == SPOOL[0]:       # Y - pay out
            self.spool.set(False)
        elif e.text().upper() == LED_ON:         # L - lamp on/off
            self.toggle_led()
        elif e.text() in ("=", "+"):             # brighter
            self.nudge_lamp(+1)
        elif e.text() in ("-", "_"):             # dimmer
            self.nudge_lamp(-1)
        elif k == Qt.Key_Escape:
            self.close()

    def keyReleaseEvent(self, e):
        if e.isAutoRepeat():
            return
        if e.key() in (Qt.Key_Up, Qt.Key_Down):
            self.btn_fwd.setDown(False)
            self.btn_back.setDown(False)
            self.send("S")

    def changeEvent(self, e):
        # Losing focus mid-press would otherwise leave a motor running.
        if e.type() == e.Type.ActivationChange and not self.isActiveWindow():
            self.stop_all()
        super().changeEvent(e)

    # -- inbound -----------------------------------------------------------
    def on_frame(self, cam: str, jpeg: bytes):
        pane = self.panes.get(cam)
        if pane is None:
            return

        if not self._use_cv:
            img = QImage.fromData(jpeg, "JPG")
            if not img.isNull():
                # Rotate here too. Without this the no-CV path showed raw
                # sensor orientation while the CV path applied ROTATE_DEG, so
                # the picture flipped depending on whether OpenCV happened to
                # be installed. Qt does the rotation so this path keeps
                # working with no cv2 available.
                i = 0 if cam == "L" else 1
                deg = ROTATE_DEG[i] % 360
                if deg:
                    img = img.transformed(QTransform().rotate(deg))
                if MIRROR_H[i]:
                    # Negative x scale = horizontal mirror.
                    img = img.transformed(QTransform().scale(-1, 1))
                pane.show_frame(img)
            return

        bgr = scv.decode(jpeg, cam)          # decode + per-camera rotation
        if bgr is None:
            return

        # Rectify on arrival so display, detection and disparity all share one
        # coordinate space. Looking up a raw-image centroid in a rectified
        # disparity map returns a confidently wrong distance.
        if self.cv and self.cv.calib.ok:
            bgr = self.cv.calib.rectify_one(bgr, cam)

        if self.cv:
            self.cv.submit(bgr if cam == "L" else None,
                           bgr if cam == "R" else None)
            if cam == "L":
                dets, _, _ = self.cv.latest()
                self._dets = dets          # the sequence reads these on tick
                if dets:
                    bgr = scv.draw_detections(bgr, dets)
                    self._det_count = len(dets)
                    self._nearest = min(
                        (d.dist_mm for d in dets if d.dist_mm is not None),
                        default=None)
                    # Screenshot AFTER drawing, so the saved image carries the
                    # outline and label. The sequence decides whether this one
                    # is worth keeping and whether it has been seen before.
                    if self.auto:
                        for d in dets:
                            self.auto.maybe_capture(bgr, d, cam)
                else:
                    self._det_count = 0
                    self._nearest = None

        pane.show_frame(bgr_to_qimage(bgr))

    def on_telemetry(self, line: str):
        vals = _TELEM_RE.findall(line)
        if vals:
            self._telem = {k: v for k, v in vals}
            self._telem_seen = time.time()

    def _set_link_text(self, t: str):
        self.link_text = t

    def _set_auto_status(self, t: str):
        self.auto_status = t

    def refresh_status(self):
        """Recompute the status strip. Still worth running when the strip is
        hidden: this is the only place the actuator countdown ages out."""
        now = time.time()

        # Drive the sequence from the ticker rather than from frame arrival,
        # so a stall in the video cannot leave a homing run going forever.
        if self.auto and self.auto.running:
            self.auto.step(self._dets, now)
        self.sync_run_buttons()

        age = now - self._telem_seen
        tel = "  ".join(f"{k}={v}" for k, v in list(self._telem.items())[:4]) \
            if self._telem and age < 3 else "telemetry: none"
        # The countdown only describes the motor. The button keeps showing the
        # state you commanded, because that is where the lining ended up.
        if now < self.act_until:
            motor = f" ({self.act_dir} {self.act_until - now:3.1f}s)"
        elif self.act_dir == "stopped":
            motor = " (stopped)"
        else:
            motor = ""

        if not SHOW_STATUS:
            return

        if self.cv_note:
            cv_txt = self.cv_note
        elif self.cv:
            _, infer_ms, depth_ms = self.cv.latest()
            near = f"{self._nearest / 10.0:.1f}cm" if self._nearest else "--"
            cv_txt = (f"cracks={self._det_count} nearest={near} "
                      f"yolo={infer_ms:.0f}ms sgbm={depth_ms:.0f}ms")
        else:
            cv_txt = "cv off"

        # Level and on/off are independent in firmware, so show both rather
        # than letting "9" imply the lamp is lit.
        lamp = (f"{self.slider.value()}" if self.led_on
                else f"off ({self.slider.value()})")

        self.status.setText(
            f"{self.link_text}   L {self.link.fps['L']:4.1f}fps   "
            f"R {self.link.fps['R']:4.1f}fps   "
            f"hook={HOOK_LABEL.get(self.hook_pos, '?')}   "
            f"lining={self.lining.label()}{motor}   "
            f"spool={self.spool.label()}   "
            f"lamp={lamp}   sent={self._last_cmd}\n"
            f"AUTO[{self.auto.state if self.auto else '-'}] "
            f"{self.auto_status}   "
            f"{cv_txt}   {tel}     "
            f"[{'/'.join(HOOK_LABEL)}]=hook  [D/R]=lining  [T/Y]=spool  "
            "[L]=lamp  [-/=]=bright  [space]=stop  [esc]=quit")

    def closeEvent(self, e):
        """Runs whether or not the status strip is shown. Without it the drive
        motor keeps running after the window shuts: link.stop() sends S and X."""
        self.ticker.stop()
        if self.cv:
            self.cv.stop()
        self.link.stop()
        super().closeEvent(e)


# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--sim", action="store_true", help="ignore hardware")
    ap.add_argument("--fullscreen", action="store_true", default=True,
                    help="fill the whole display (default)")
    ap.add_argument("--maximised", "--maximized", dest="fullscreen",
                    action="store_false",
                    help="fill the screen but keep the menu bar and Dock")
    ap.add_argument("--windowed", action="store_true",
                    help="fixed 1280x960 window, for layout tweaking")
    ap.add_argument("--width", type=int, default=None,
                    help="explicit width; height follows the 4:3 design")
    ap.add_argument("--no-cv", action="store_true",
                    help="skip YOLO and stereo depth")
    ap.add_argument("--conf", type=float, default=0.4,
                    help="detection confidence threshold")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    if args.list:
        if not HAVE_SERIAL:
            print("pyserial not installed:  pip3 install pyserial")
        else:
            for p in list_ports.comports():
                print(f"{p.device:32s} {p.description}")
        return 0

    port = None if args.sim else (args.port or autodetect_port())
    if not args.sim and not port:
        print("No serial port found — starting in SIM mode. "
              "Use --list to see ports, or --port to pick one.")

    problems = check_assets()
    if problems:
        print("Artwork problems — fix ASSETS at the top of this file:")
        print("\n".join(problems))
        return 1

    if not HAVE_CV and not args.no_cv:
        print(f"CV disabled ({CV_IMPORT_ERROR}) — video only. "
              "pip3 install opencv-python ultralytics")

    app = QApplication(sys.argv)

    # availableGeometry excludes the macOS menu bar and Dock, so a --windowed
    # panel is never partly off-screen; fullscreen uses the whole display.
    screen = app.primaryScreen()
    if args.windowed:
        win = CANVAS
    elif args.width:
        win = QSize(args.width, int(args.width * CANVAS.height() / CANVAS.width()))
    else:
        geo = screen.geometry() if args.fullscreen else screen.availableGeometry()
        win = geo.size()

    panel = Panel(SerialLink(port, args.baud), use_cv=not args.no_cv,
                  conf=args.conf, win_size=win)
    panel.link.start()
    if args.fullscreen:
        panel.showFullScreen()
    else:
        panel.show()
    panel.raise_()
    panel.activateWindow()
    panel.setFocus()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
