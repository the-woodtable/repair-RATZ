#!/usr/bin/env python3
"""
rat #88 - showcase control panel (PySide6)

Speaks the SAME protocol as control_panel_stereo2.py, so it is a drop-in
alternative front end. Keep the old panel for diagnostics and recording; use
this one when people are watching.

    pip3 install PySide6
    pip3 install pyserial          # optional - runs in SIM mode without it

    python3 rat88_panel.py                       # auto-detect port
    python3 rat88_panel.py --sim                 # no hardware, animated feed
    python3 rat88_panel.py --port /dev/cu.usbserial-XXXX
    python3 rat88_panel.py --list                # show ports and exit

Wire format in  (from stereo_serial.py / main.cpp):
    0xAA 0x55 | ID(1) | uint32 LE length | payload
    ID 0x00 or 'L' = left camera JPEG
    ID 0x01 or 'R' = right camera JPEG
    ID 0x02        = telemetry (JSON-ish text)

Wire format out (single characters, no terminator - what main.cpp listens for):
    F/B/S      drive forward / back / stop
    D/R/X      actuator deploy / retract / stop
    H/U        hook / release
    A          assembly
    L/O        LED on / off
    Z          zero odometry
    '0'-'9'    lamp brightness, mapped to 0-255 in firmware

NOTE: the current firmware has NO dead-man watchdog. A motor runs until it is
told to stop, so every press-and-hold control sends its stop character on
release, on mouse-leave, and on window deactivate.
"""

import argparse
import math
import queue
import re
import struct
import sys
import threading
import time

from PySide6.QtCore import (QObject, QPoint, QPointF, QRect, QSize, Qt, QTimer,
                            Signal)
from PySide6.QtGui import (QBrush, QColor, QFont, QIcon, QImage, QPainter,
                           QPainterPath, QPen, QPixmap, QPolygonF)
from PySide6.QtWidgets import (QApplication, QLabel, QPushButton, QSlider,
                               QWidget)

try:
    import serial
    from serial.tools import list_ports
    HAVE_SERIAL = True
except ImportError:
    HAVE_SERIAL = False


# ---------------------------------------------------------------------------
# ASSETS - point these at your exported artwork. None = generated placeholder.
# ---------------------------------------------------------------------------

ASSETS = {
    "background":   None,   # e.g. "art/bg.png"   full 1280x960 plate
    "arrow_up":     None,
    "arrow_down":   None,
    "cheese":       None,   # slider handle
    "groove":       None,   # slider bar
    "can":          None,   # DEPLOY LINING
    "hook":         None,
}

# ---------------------------------------------------------------------------
# Design constants - measured off the mockup, 1280x960 canvas
# ---------------------------------------------------------------------------

CANVAS = QSize(1280, 960)

GEO = {
    "video_left":   QRect(90,  176, 376, 502),
    "video_right":  QRect(473, 176, 376, 502),
    "arrow_up":     QRect(966, 349, 122, 131),
    "arrow_down":   QRect(966, 505, 122, 136),
    "slider":       QRect(424, 744, 342,  78),
    "can":          QRect(838, 729, 155, 120),
    "hook":         QRect(1068, 722,  88, 133),
    "title":        QRect(0,    50, 1280, 100),
    "status":       QRect(0,   904, 1280,  50),
}

C_BG        = QColor("#E4E2DD")
C_INK       = QColor("#1B4EA0")
C_BLUE      = QColor("#7BA3D8")
C_RED       = QColor("#C1666B")
C_PAW       = QColor("#CFCFCB")
C_CHEESE    = QColor("#EDB94F")
C_HOLE      = QColor("#E0913F")
C_METAL     = QColor("#C9CDD2")

# Protocol
MAGIC = b"\xAA\x55"
ID_MAP = {0x00: "L", 0x4C: "L",     # 0x4C = 'L'
          0x01: "R", 0x52: "R"}     # 0x52 = 'R'
TELEM_ID = 0x02
MAX_FRAME = 200_000
SOI, EOI = b"\xFF\xD8", b"\xFF\xD9"

# Set True only if the hook physically fouls the wheels when deployed.
INTERLOCK_HOOK_BLOCKS_DRIVE = False

_TELEM_RE = re.compile(r'"?(\w+)"?\s*[:=]\s*(-?\d+(?:\.\d+)?)')


# ---------------------------------------------------------------------------
# Placeholder art
# ---------------------------------------------------------------------------

def _blank(size: QSize) -> QPixmap:
    pm = QPixmap(size)
    pm.fill(Qt.transparent)
    return pm


def _painter(pm: QPixmap) -> QPainter:
    p = QPainter(pm)
    p.setRenderHint(QPainter.Antialiasing, True)
    p.setPen(Qt.NoPen)
    return p


def make_triangle(size: QSize, up: bool, color: QColor) -> QPixmap:
    pm = _blank(size)
    p = _painter(pm)
    w, h, m = size.width(), size.height(), 2.0
    if up:
        pts = [QPointF(w / 2, m), QPointF(w - m, h - m), QPointF(m, h - m)]
    else:
        pts = [QPointF(m, m), QPointF(w - m, m), QPointF(w / 2, h - m)]
    p.setBrush(QBrush(color))
    p.drawPolygon(QPolygonF(pts))
    p.end()
    return pm


def make_cheese(size: QSize) -> QPixmap:
    pm = _blank(size)
    p = _painter(pm)
    w, h = size.width(), size.height()
    path = QPainterPath()
    path.moveTo(w * 0.86, h * 0.06)
    path.lineTo(w * 0.98, h * 0.62)
    path.lineTo(w * 0.02, h * 0.96)
    path.closeSubpath()
    p.setBrush(QBrush(C_CHEESE))
    p.drawPath(path)
    p.setBrush(QBrush(C_HOLE))
    for fx, fy, fr in ((0.30, 0.72, 0.075), (0.52, 0.55, 0.055),
                       (0.70, 0.38, 0.048), (0.46, 0.80, 0.042)):
        p.drawEllipse(QPoint(int(w * fx), int(h * fy)), int(w * fr), int(w * fr))
    p.end()
    return pm


def make_groove(size: QSize) -> QPixmap:
    pm = _blank(size)
    p = _painter(pm)
    p.setBrush(QBrush(C_BLUE))
    p.drawRoundedRect(0, 0, size.width(), size.height(), 3, 3)
    p.end()
    return pm


def make_can(size: QSize, label="DEPLOY\nLINING") -> QPixmap:
    pm = _blank(size)
    p = _painter(pm)
    w, h = size.width(), size.height()
    lid_h = int(h * 0.20)
    p.setBrush(QBrush(C_METAL))
    p.drawRoundedRect(0, lid_h // 2, w, h - lid_h // 2, 10, 10)
    p.setBrush(QBrush(C_BLUE))
    p.drawEllipse(0, 0, w, lid_h)
    p.setBrush(QBrush(C_METAL.lighter(108)))
    p.drawEllipse(int(w * 0.08), int(lid_h * 0.22), int(w * 0.84), int(lid_h * 0.60))
    p.setPen(QPen(C_INK))
    p.setFont(QFont("Arial", 15, QFont.Bold))
    p.drawText(QRect(0, lid_h, w, h - lid_h), Qt.AlignCenter, label)
    p.end()
    return pm


def make_hook(size: QSize) -> QPixmap:
    pm = _blank(size)
    p = _painter(pm)
    w, h = size.width(), size.height()
    t = w * 0.34
    p.setBrush(QBrush(C_BLUE))
    p.drawRect(0, 0, int(t), int(h))                    # vertical shaft
    p.drawRect(0, 0, int(w), int(t * 0.75))             # top bar
    p.drawRect(int(w - t), 0, int(t), int(h * 0.42))    # short right leg
    p.end()
    return pm


def make_paw(size: QSize, flip=False) -> QPixmap:
    pm = _blank(size)
    p = _painter(pm)
    w, h = size.width(), size.height()
    p.setBrush(QBrush(C_PAW))
    if flip:
        p.translate(w, 0)
        p.scale(-1, 1)
    p.drawEllipse(int(w * 0.18), int(h * 0.38), int(w * 0.74), int(h * 0.58))
    for fx, fy, fw, fh in ((0.02, 0.30, 0.30, 0.34), (0.26, 0.06, 0.28, 0.32),
                           (0.56, 0.02, 0.26, 0.30), (0.80, 0.18, 0.22, 0.28)):
        p.drawEllipse(int(w * fx), int(h * fy), int(w * fw), int(h * fh))
    p.end()
    return pm


def make_background(_size=None) -> QPixmap:
    pm = QPixmap(CANVAS)
    pm.fill(C_BG)
    p = _painter(pm)
    p.drawPixmap(1090, 10, make_paw(QSize(230, 240)))
    p.drawPixmap(60, 690, make_paw(QSize(230, 250), flip=True))
    p.end()
    return pm


def load_or(key: str, fallback, size: QSize) -> QPixmap:
    path = ASSETS.get(key)
    if path:
        pm = QPixmap(path)
        if not pm.isNull():
            return pm.scaled(size, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        print(f"[assets] could not load {path!r}; using placeholder for {key}")
    return fallback(size)


# ---------------------------------------------------------------------------
# Widgets
# ---------------------------------------------------------------------------

class ArtButton(QPushButton):
    """A button that is exactly its artwork.

    Clicks only land on non-transparent pixels, so the two touching arrows and
    the hook's empty notch behave the way they look. Because the firmware has
    no watchdog, leaving the button while held also fires `released` - a motor
    left running is worse than a jerky UI.
    """

    def __init__(self, pixmap: QPixmap, rect: QRect, parent=None):
        super().__init__(parent)
        scaled = pixmap.scaled(rect.size(), Qt.IgnoreAspectRatio,
                               Qt.SmoothTransformation)
        self._normal = scaled
        self._hover = self._tint(scaled, 1.12)
        self._down = self._tint(scaled, 0.86)
        self.setGeometry(rect)
        self.setIconSize(rect.size())
        self.setIcon(QIcon(self._normal))
        self.setCursor(Qt.PointingHandCursor)
        self.setFocusPolicy(Qt.NoFocus)
        self.setStyleSheet("border:none; background:transparent;")
        mask = scaled.mask()
        if not mask.isNull():
            self.setMask(mask)

    @staticmethod
    def _tint(pm: QPixmap, factor: float) -> QPixmap:
        img = pm.toImage().convertToFormat(QImage.Format_ARGB32)
        for y in range(img.height()):
            for x in range(img.width()):
                c = QColor.fromRgba(img.pixel(x, y))
                if c.alpha() == 0:
                    continue
                c = c.lighter(int(factor * 100)) if factor >= 1 \
                    else c.darker(int(100 / factor))
                img.setPixel(x, y, c.rgba())
        return QPixmap.fromImage(img)

    def enterEvent(self, e):
        if not self.isDown():
            self.setIcon(QIcon(self._hover))
        super().enterEvent(e)

    def leaveEvent(self, e):
        if self.isDown():
            self.setDown(False)
            self.released.emit()      # safety stop
        self.setIcon(QIcon(self._normal))
        super().leaveEvent(e)

    def mousePressEvent(self, e):
        self.setIcon(QIcon(self._down))
        super().mousePressEvent(e)

    def mouseReleaseEvent(self, e):
        self.setIcon(QIcon(self._hover if self.underMouse() else self._normal))
        super().mouseReleaseEvent(e)


class ImageSlider(QSlider):
    """Horizontal slider drawn from a groove pixmap and a handle pixmap.
    Click anywhere to jump, drag to scrub."""

    def __init__(self, handle: QPixmap, groove: QPixmap, rect: QRect,
                 lo=0, hi=9, parent=None):
        super().__init__(Qt.Horizontal, parent)
        self._handle = handle
        self._groove = groove
        self.setGeometry(rect)
        self.setRange(lo, hi)
        self.setValue(hi)
        self.setCursor(Qt.PointingHandCursor)
        self.setFocusPolicy(Qt.NoFocus)

    def _travel(self) -> int:
        return max(1, self.width() - self._handle.width())

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        gx = self._handle.width() // 2
        gy = (self.height() - self._groove.height()) // 2
        p.drawPixmap(gx, gy, self._groove.scaled(
            max(1, self.width() - gx), self._groove.height(),
            Qt.IgnoreAspectRatio, Qt.SmoothTransformation))
        frac = (self.value() - self.minimum()) / max(1, self.maximum() - self.minimum())
        hy = (self.height() - self._handle.height()) // 2
        p.drawPixmap(int(frac * self._travel()), hy, self._handle)
        p.end()

    def _set_from_x(self, x: float):
        frac = (x - self._handle.width() / 2) / self._travel()
        frac = min(1.0, max(0.0, frac))
        self.setValue(round(self.minimum() + frac * (self.maximum() - self.minimum())))

    def mousePressEvent(self, e):
        self._set_from_x(e.position().x())

    def mouseMoveEvent(self, e):
        self._set_from_x(e.position().x())


class VideoPane(QLabel):
    def __init__(self, rect: QRect, active: bool, parent=None):
        super().__init__(parent)
        self.setGeometry(rect)
        self.setAlignment(Qt.AlignCenter)
        self.set_active(active)

    def set_active(self, active: bool):
        border = f"2px solid {C_BLUE.name()}" if active else "none"
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

    frame_ready = Signal(str, QImage)     # "L"/"R", image
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

    def _parse(self):
        while True:
            i = self._buf.find(MAGIC)
            if i < 0:
                if len(self._buf) > 1:
                    del self._buf[:-1]          # keep a possible split magic
                return
            if i:
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
            img = QImage.fromData(payload[:end + 2], "JPG")
            if not img.isNull():
                self._tick(cam)
                self.frame_ready.emit(cam, img)

    def _tick(self, cam):
        now = time.time()
        dt = now - self._last_t[cam]
        self._last_t[cam] = now
        if dt > 0:
            self.fps[cam] = 0.85 * self.fps[cam] + 0.15 * (1.0 / dt)

    def _sim(self):
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
                p.drawText(int(cx - 34), int(cy - 27), "crack 0.87")
                p.setPen(QPen(QColor("#8899AA"), 1))
                p.drawText(8, 16, f"SIMULATED {cam}")
                p.end()
                self._tick(cam)
                self.frame_ready.emit(cam, img.copy())
            time.sleep(1 / 20)


# ---------------------------------------------------------------------------
# Panel
# ---------------------------------------------------------------------------

class Panel(QWidget):

    def __init__(self, link: SerialLink):
        super().__init__()
        self.link = link
        self.setWindowTitle("rat #88")
        self.setFixedSize(CANVAS)
        self.setFocusPolicy(Qt.StrongFocus)      # needed for arrow-key driving

        self.hook_out = False
        self._led_last = None
        self._last_cmd = "-"
        self._telem = {}
        self._telem_seen = 0.0
        self.link_text = "starting"

        self._build_background()
        self._build_video()
        self._build_controls()
        self._build_status()

        link.frame_ready.connect(self.on_frame)
        link.telemetry.connect(self.on_telemetry)
        link.link_state.connect(self._set_link_text)

        self.ticker = QTimer(self)
        self.ticker.timeout.connect(self.refresh_status)
        self.ticker.start(200)

    # -- construction ------------------------------------------------------
    def _build_background(self):
        pm = load_or("background", make_background, CANVAS)
        self.bg = QLabel(self)
        self.bg.setGeometry(QRect(QPoint(0, 0), CANVAS))
        self.bg.setPixmap(pm.scaled(CANVAS, Qt.IgnoreAspectRatio,
                                    Qt.SmoothTransformation))

        self.title = QLabel("rat #88", self)
        self.title.setGeometry(GEO["title"])
        self.title.setAlignment(Qt.AlignCenter)
        self.title.setStyleSheet(
            f"color:{C_INK.name()}; font-family:Arial; font-size:64px;"
            "font-weight:800; background:transparent;")

    def _build_video(self):
        self.panes = {
            "L": VideoPane(GEO["video_left"], active=True, parent=self),
            "R": VideoPane(GEO["video_right"], active=False, parent=self),
        }

    def _build_controls(self):
        g = GEO
        up = load_or("arrow_up", lambda s: make_triangle(s, True, C_RED), g["arrow_up"].size())
        dn = load_or("arrow_down", lambda s: make_triangle(s, False, C_RED), g["arrow_down"].size())

        self.btn_fwd = ArtButton(up, g["arrow_up"], self)
        self.btn_fwd.setToolTip("Drive forward — hold, or Up arrow key")
        self.btn_fwd.pressed.connect(lambda: self.drive("F"))
        self.btn_fwd.released.connect(lambda: self.send("S"))

        self.btn_back = ArtButton(dn, g["arrow_down"], self)
        self.btn_back.setToolTip("Drive back — hold, or Down arrow key")
        self.btn_back.pressed.connect(lambda: self.drive("B"))
        self.btn_back.released.connect(lambda: self.send("S"))

        can = load_or("can", make_can, g["can"].size())
        self.btn_act = ArtButton(can, g["can"], self)
        self.btn_act.setToolTip("Deploy lining — hold to extend, release to stop")
        self.btn_act.pressed.connect(lambda: self.send("D"))
        self.btn_act.released.connect(lambda: self.send("X"))

        hook = load_or("hook", make_hook, g["hook"].size())
        self.btn_hook = ArtButton(hook, g["hook"], self)
        self.btn_hook.setToolTip("Click to deploy hook, click again to release")
        self.btn_hook.clicked.connect(self.toggle_hook)

        handle = load_or("cheese", make_cheese, QSize(100, 72))
        groove = load_or("groove", make_groove, QSize(242, 26))
        # Firmware maps the characters '0'-'9' onto 0-255, so the slider is 0-9
        # and one character carries the level.
        self.slider = ImageSlider(handle, groove, g["slider"], 0, 9, self)
        self.slider.setToolTip("Lamp brightness (0-9)")
        self.slider.valueChanged.connect(self.on_led)

    def _build_status(self):
        self.status = QLabel(self)
        self.status.setGeometry(GEO["status"])
        self.status.setAlignment(Qt.AlignCenter)
        self.status.setStyleSheet(
            f"color:{C_INK.name()}; font-family:Menlo,Consolas,monospace;"
            "font-size:12px; background:transparent;")

    # -- commands ----------------------------------------------------------
    def send(self, ch: str):
        self.link.send(ch)
        self._last_cmd = ch

    def drive(self, ch: str):
        if INTERLOCK_HOOK_BLOCKS_DRIVE and self.hook_out:
            self._last_cmd = "blocked: hook out"
            return
        self.send(ch)

    def toggle_hook(self):
        self.hook_out = not self.hook_out
        self.send("H" if self.hook_out else "U")

    def on_led(self, v: int):
        # Only send on change; a drag otherwise floods the port and competes
        # with the video for bandwidth.
        if v != self._led_last:
            self._led_last = v
            self.send(str(v))

    def stop_all(self):
        self.send("S")
        self.link.send("X")

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
        elif k == Qt.Key_Z:
            self.send("Z")
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
    def on_frame(self, cam: str, img: QImage):
        # Plug crack_tracker.py in here if you want boxes on the showcase feed.
        pane = self.panes.get(cam)
        if pane:
            pane.show_frame(img)

    def on_telemetry(self, line: str):
        vals = _TELEM_RE.findall(line)
        if vals:
            self._telem = {k: v for k, v in vals}
            self._telem_seen = time.time()

    def _set_link_text(self, t: str):
        self.link_text = t

    def refresh_status(self):
        age = time.time() - self._telem_seen
        tel = "  ".join(f"{k}={v}" for k, v in list(self._telem.items())[:4]) \
            if self._telem and age < 3 else "telemetry: none"
        self.status.setText(
            f"{self.link_text}   L {self.link.fps['L']:4.1f}fps   "
            f"R {self.link.fps['R']:4.1f}fps   "
            f"hook={'OUT' if self.hook_out else 'IN'}   "
            f"lamp={self.slider.value()}   sent={self._last_cmd}   {tel}"
            "        [space]=stop all  [esc]=quit")

    def closeEvent(self, e):
        self.ticker.stop()
        self.link.stop()
        super().closeEvent(e)


# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--sim", action="store_true", help="ignore hardware")
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

    app = QApplication(sys.argv)
    link = SerialLink(port, args.baud)
    panel = Panel(link)
    link.start()
    panel.show()
    panel.raise_()
    panel.activateWindow()
    panel.setFocus()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
