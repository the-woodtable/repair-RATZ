#!/usr/bin/env python3
"""
rat #88 - control panel (placeholder art build)

Runs with zero assets: every graphic is generated in code so you can see the
layout working immediately. To swap in your own artwork, edit ASSETS below and
point each key at a PNG/SVG file. Anything left as None stays a placeholder.

    pip install PySide6
    pip install pyserial      # optional - runs in SIM mode without it

    python rat88_panel.py                 # sim mode, no hardware needed
    python rat88_panel.py --port COM5     # real ESP32 over USB
    python rat88_panel.py --list          # show available serial ports
"""

import argparse
import math
import queue
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
# ASSETS - point these at your exported images when you have them.
# Sizes are taken from the geometry table below, so exports should match those
# dimensions (or be larger; they get scaled down smoothly).
# ---------------------------------------------------------------------------

ASSETS = {
    "background":   None,   # e.g. "art/bg.png"      full 1280x960 plate
    "arrow_up":     None,   # e.g. "art/up.png"
    "arrow_down":   None,
    "cheese":       None,   # slider handle
    "groove":       None,   # slider bar
    "can":          None,   # DEPLOY LINING
    "hook":         None,
}


# ---------------------------------------------------------------------------
# Design constants - straight from the mockup, 1280x960 canvas
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
    "status":       QRect(0,   916, 1280,  44),
}

C_BG        = QColor("#E4E2DD")
C_INK       = QColor("#1B4EA0")
C_BLUE      = QColor("#7BA3D8")
C_RED       = QColor("#C1666B")
C_PAW       = QColor("#CFCFCB")
C_CHEESE    = QColor("#EDB94F")
C_CHEESE_HL = QColor("#F5CC72")
C_HOLE      = QColor("#E0913F")
C_METAL     = QColor("#C9CDD2")

# Command protocol - line based ASCII, newline terminated
CMD_KEEPALIVE = "KA"
KEEPALIVE_MS  = 100     # must be well under the firmware watchdog (~400ms)
ACK_TIMEOUT_MS = 500
SLIDER_THROTTLE_MS = 50

MAGIC = 0xAA55
TYPE_JPEG = 0x01
TYPE_TELEM = 0x02


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
    w, h = size.width(), size.height()
    m = 2.0
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
    f = QFont("Arial", 15, QFont.Bold)
    p.setFont(f)
    p.drawText(QRect(0, lid_h, w, h - lid_h), Qt.AlignCenter, label)
    p.end()
    return pm


def make_hook(size: QSize) -> QPixmap:
    pm = _blank(size)
    p = _painter(pm)
    w, h = size.width(), size.height()
    t = w * 0.34                      # stroke thickness
    p.setBrush(QBrush(C_BLUE))
    p.drawRect(0, 0, int(t), int(h))              # vertical shaft
    p.drawRect(0, 0, int(w), int(t * 0.75))       # top horizontal
    p.drawRect(int(w - t), 0, int(t), int(h * 0.42))  # short right leg
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


def make_background() -> QPixmap:
    pm = QPixmap(CANVAS)
    pm.fill(C_BG)
    p = _painter(pm)
    p.drawPixmap(1090, 10, make_paw(QSize(230, 240), flip=False))
    p.drawPixmap(60, 700, make_paw(QSize(230, 250), flip=True))
    p.end()
    return pm


def load_or(key: str, fallback, size: QSize) -> QPixmap:
    """Use the file in ASSETS[key] if set, otherwise the generated placeholder."""
    path = ASSETS.get(key)
    if path:
        pm = QPixmap(path)
        if not pm.isNull():
            return pm.scaled(size, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        print(f"[assets] could not load {path!r}, using placeholder for {key}")
    return fallback(size)


# ---------------------------------------------------------------------------
# Widgets
# ---------------------------------------------------------------------------

class ArtButton(QPushButton):
    """A button that is exactly its artwork.

    Clicks only register on non-transparent pixels, so shapes that touch (the
    two arrows) or have big empty notches (the hook) do not steal each other's
    clicks. Hover and pressed states are generated from the base pixmap.
    """

    def __init__(self, pixmap: QPixmap, rect: QRect, parent=None):
        super().__init__(parent)
        self._normal = pixmap
        self._hover = self._tint(pixmap, 1.12)
        self._down = self._tint(pixmap, 0.86)
        self.setGeometry(rect)
        self.setIconSize(rect.size())
        self.setIcon(QIcon(self._normal))
        self.setCursor(Qt.PointingHandCursor)
        self.setFocusPolicy(Qt.NoFocus)
        self.setStyleSheet("border:none; background:transparent;")
        mask = pixmap.scaled(rect.size(), Qt.IgnoreAspectRatio,
                             Qt.SmoothTransformation).mask()
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
                c = c.lighter(int(factor * 100)) if factor >= 1 else c.darker(int(100 / factor))
                img.setPixel(x, y, c.rgba())
        return QPixmap.fromImage(img)

    def enterEvent(self, e):
        self.setIcon(QIcon(self._hover))
        super().enterEvent(e)

    def leaveEvent(self, e):
        self.setIcon(QIcon(self._normal))
        super().leaveEvent(e)

    def mousePressEvent(self, e):
        self.setIcon(QIcon(self._down))
        super().mousePressEvent(e)

    def mouseReleaseEvent(self, e):
        self.setIcon(QIcon(self._hover if self.underMouse() else self._normal))
        super().mouseReleaseEvent(e)


class ImageSlider(QSlider):
    """Horizontal slider drawn from two pixmaps: a groove and a handle.

    Click-anywhere-to-jump plus drag, which feels better than Qt's default
    page-step behaviour for a demo panel.
    """

    def __init__(self, handle: QPixmap, groove: QPixmap, rect: QRect, parent=None):
        super().__init__(Qt.Horizontal, parent)
        self._handle = handle
        self._groove = groove
        self.setGeometry(rect)
        self.setRange(0, 255)
        self.setValue(0)
        self.setCursor(Qt.PointingHandCursor)
        self.setFocusPolicy(Qt.NoFocus)

    def _travel(self) -> int:
        return max(1, self.width() - self._handle.width())

    def _handle_x(self) -> int:
        frac = (self.value() - self.minimum()) / max(1, self.maximum() - self.minimum())
        return int(frac * self._travel())

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        gy = (self.height() - self._groove.height()) // 2
        gx = self._handle.width() // 2
        p.drawPixmap(gx, gy, self._groove.scaled(
            self.width() - gx, self._groove.height(),
            Qt.IgnoreAspectRatio, Qt.SmoothTransformation))
        hy = (self.height() - self._handle.height()) // 2
        p.drawPixmap(self._handle_x(), hy, self._handle)
        p.end()

    def _set_from_x(self, x: int):
        frac = (x - self._handle.width() / 2) / self._travel()
        frac = min(1.0, max(0.0, frac))
        self.setValue(int(self.minimum() + frac * (self.maximum() - self.minimum())))

    def mousePressEvent(self, e):
        self._set_from_x(e.position().x())
        self.sliderPressed.emit()

    def mouseMoveEvent(self, e):
        self._set_from_x(e.position().x())

    def mouseReleaseEvent(self, e):
        self.sliderReleased.emit()


class VideoPane(QLabel):
    """Black pane that shows frames. active=True draws the blue selection border."""

    def __init__(self, rect: QRect, active: bool, parent=None):
        super().__init__(parent)
        self.setGeometry(rect)
        self.setAlignment(Qt.AlignCenter)
        self.set_active(active)

    def set_active(self, active: bool):
        border = f"2px solid {C_BLUE.name()}" if active else "none"
        self.setStyleSheet(f"background:#000000; border:{border};")

    def show_frame(self, img: QImage):
        pm = QPixmap.fromImage(img).scaled(
            self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.setPixmap(pm)


# ---------------------------------------------------------------------------
# Serial layer
# ---------------------------------------------------------------------------

class SerialLink(QObject):
    """Owns the port. Nothing else in the app touches it.

    A reader thread parses the framed stream and emits Qt signals (queued
    automatically across threads, so handlers run on the GUI thread). A writer
    thread drains a queue so no button handler can ever block on serial IO.
    """

    frame_ready = Signal(QImage)
    telemetry = Signal(str)
    link_state = Signal(str)

    def __init__(self, port: str | None, baud: int = 921600, parent=None):
        super().__init__(parent)
        self.port_name = port
        self.baud = baud
        self.ser = None
        self.simulated = True
        self._tx = queue.Queue()
        self._stop = threading.Event()
        self._buf = bytearray()
        self._last_frame_t = time.time()
        self.fps = 0.0

    # -- lifecycle ---------------------------------------------------------
    def start(self):
        if self.port_name and HAVE_SERIAL:
            try:
                self.ser = serial.Serial(self.port_name, self.baud, timeout=0.05)
                self.simulated = False
                self.link_state.emit(f"LINK  {self.port_name} @ {self.baud}")
            except Exception as exc:
                self.link_state.emit(f"SIM  (open failed: {exc})")
        else:
            reason = "no port given" if not self.port_name else "pyserial missing"
            self.link_state.emit(f"SIM  ({reason})")

        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._writer, daemon=True).start()

    def stop(self):
        self._stop.set()
        self.send("DRV:S")
        self.send("ACT:S")
        self.send("HOK:S")
        time.sleep(0.1)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass

    # -- tx ----------------------------------------------------------------
    def send(self, cmd: str):
        self._tx.put(cmd)

    def _writer(self):
        while not self._stop.is_set():
            try:
                cmd = self._tx.get(timeout=0.1)
            except queue.Empty:
                continue
            if self.ser:
                try:
                    self.ser.write((cmd + "\n").encode())
                except Exception as exc:
                    self.link_state.emit(f"TX ERROR  {exc}")
            elif cmd != CMD_KEEPALIVE:
                # sim mode: echo an ack so the UI state machine can be tested
                self.telemetry.emit(f"ACK:{cmd}")

    # -- rx ----------------------------------------------------------------
    def _reader(self):
        if self.ser:
            self._read_serial()
        else:
            self._read_sim()

    def _read_serial(self):
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(4096)
            except Exception as exc:
                self.link_state.emit(f"RX ERROR  {exc}")
                return
            if chunk:
                self._buf += chunk
                self._parse()

    def _parse(self):
        """Frame format:  AA 55 | type(1) | length(4, LE) | payload"""
        while True:
            i = self._buf.find(b"\x55\xAA")          # 0xAA55 little-endian on wire
            if i < 0:
                if len(self._buf) > 2:
                    del self._buf[:-2]
                return
            if i:
                del self._buf[:i]
            if len(self._buf) < 7:
                return
            ftype = self._buf[2]
            (length,) = struct.unpack_from("<I", self._buf, 3)
            if length > 4_000_000:                   # desync guard
                del self._buf[:2]
                continue
            if len(self._buf) < 7 + length:
                return
            payload = bytes(self._buf[7:7 + length])
            del self._buf[:7 + length]
            if ftype == TYPE_JPEG:
                img = QImage.fromData(payload, "JPG")
                if not img.isNull():
                    self._tick_fps()
                    self.frame_ready.emit(img)
            elif ftype == TYPE_TELEM:
                for line in payload.decode(errors="replace").splitlines():
                    if line.strip():
                        self.telemetry.emit(line.strip())

    def _tick_fps(self):
        now = time.time()
        dt = now - self._last_frame_t
        self._last_frame_t = now
        if dt > 0:
            self.fps = 0.85 * self.fps + 0.15 * (1.0 / dt)

    def _read_sim(self):
        """Animated test pattern so the panel is alive without hardware."""
        w, h = 320, 240
        t0 = time.time()
        while not self._stop.is_set():
            t = time.time() - t0
            img = QImage(w, h, QImage.Format_RGB888)
            img.fill(QColor(14, 14, 18))
            p = QPainter(img)
            p.setRenderHint(QPainter.Antialiasing, True)
            p.setPen(QPen(QColor(40, 44, 52), 1))
            for gx in range(0, w, 32):
                p.drawLine(gx, 0, gx, h)
            for gy in range(0, h, 32):
                p.drawLine(0, gy, w, gy)
            cx = w / 2 + math.sin(t * 1.1) * w * 0.28
            cy = h / 2 + math.cos(t * 0.7) * h * 0.24
            p.setPen(QPen(QColor("#00E676"), 2))
            p.setBrush(Qt.NoBrush)
            p.drawRect(int(cx - 34), int(cy - 22), 68, 44)
            p.setFont(QFont("Arial", 9))
            p.drawText(int(cx - 34), int(cy - 27), "crack 0.87")
            p.setPen(QPen(QColor("#8899AA"), 1))
            p.drawText(8, 16, "SIMULATED FEED")
            p.end()
            self._tick_fps()
            self.frame_ready.emit(img.copy())
            time.sleep(1 / 20)


# ---------------------------------------------------------------------------
# Main panel
# ---------------------------------------------------------------------------

class Panel(QWidget):

    def __init__(self, link: SerialLink):
        super().__init__()
        self.link = link
        self.setWindowTitle("rat #88")
        self.setFixedSize(CANVAS)
        self.setFocusPolicy(Qt.StrongFocus)   # required for arrow-key driving

        self.pending = {}          # channel -> (cmd, sent_at)
        self.acked = {}            # channel -> cmd
        self.hook_deployed = False
        self._last_led_send = 0.0
        self._last_status = "ready"

        self._build_background()
        self._build_video()
        self._build_controls()
        self._build_status()

        self.link.frame_ready.connect(self.on_frame)
        self.link.telemetry.connect(self.on_telemetry)
        self.link.link_state.connect(self.set_link_text)

        self.keepalive = QTimer(self)
        self.keepalive.timeout.connect(lambda: self.link.send(CMD_KEEPALIVE))
        self.keepalive.start(KEEPALIVE_MS)

        self.ui_tick = QTimer(self)
        self.ui_tick.timeout.connect(self.refresh_status)
        self.ui_tick.start(200)

    # -- construction ------------------------------------------------------
    def _build_background(self):
        bg = ASSETS.get("background")
        pm = QPixmap(bg) if bg else QPixmap()
        if pm.isNull():
            pm = make_background()
        self.bg = QLabel(self)
        self.bg.setGeometry(QRect(QPoint(0, 0), CANVAS))
        self.bg.setPixmap(pm.scaled(CANVAS, Qt.IgnoreAspectRatio, Qt.SmoothTransformation))

        self.title = QLabel("rat #88", self)
        self.title.setGeometry(GEO["title"])
        self.title.setAlignment(Qt.AlignCenter)
        self.title.setStyleSheet(
            f"color:{C_INK.name()}; font-family:Arial; font-size:64px; font-weight:800;"
            "background:transparent;")

    def _build_video(self):
        self.pane_left = VideoPane(GEO["video_left"], active=True, parent=self)
        self.pane_right = VideoPane(GEO["video_right"], active=False, parent=self)

    def _build_controls(self):
        g = GEO

        up = load_or("arrow_up", lambda s: make_triangle(s, True, C_RED), g["arrow_up"].size())
        dn = load_or("arrow_down", lambda s: make_triangle(s, False, C_RED), g["arrow_down"].size())
        self.btn_fwd = ArtButton(up, g["arrow_up"], self)
        self.btn_back = ArtButton(dn, g["arrow_down"], self)
        self.btn_fwd.setToolTip("Drive forward (hold, or Up arrow key)")
        self.btn_back.setToolTip("Drive back (hold, or Down arrow key)")
        # hold-to-move: clicked would only fire on release
        self.btn_fwd.pressed.connect(lambda: self.drive("F"))
        self.btn_fwd.released.connect(lambda: self.drive("S"))
        self.btn_back.pressed.connect(lambda: self.drive("B"))
        self.btn_back.released.connect(lambda: self.drive("S"))

        can = load_or("can", make_can, g["can"].size())
        self.btn_actuator = ArtButton(can, g["can"], self)
        self.btn_actuator.setToolTip("Deploy lining (hold to extend)")
        self.btn_actuator.pressed.connect(lambda: self.command("ACT", "D"))
        self.btn_actuator.released.connect(lambda: self.command("ACT", "S"))

        hook = load_or("hook", make_hook, g["hook"].size())
        self.btn_hook = ArtButton(hook, g["hook"], self)
        self.btn_hook.setToolTip("Toggle hook deploy / retract")
        self.btn_hook.clicked.connect(self.toggle_hook)

        handle = load_or("cheese", make_cheese, QSize(100, 72))
        groove = load_or("groove", make_groove, QSize(242, 26))
        self.slider_led = ImageSlider(handle, groove, g["slider"], self)
        self.slider_led.setToolTip("LED brightness")
        self.slider_led.valueChanged.connect(self.on_led_changed)
        self.slider_led.sliderReleased.connect(self.on_led_released)

    def _build_status(self):
        self.status = QLabel(self)
        self.status.setGeometry(GEO["status"])
        self.status.setAlignment(Qt.AlignCenter)
        self.status.setStyleSheet(
            f"color:{C_INK.name()}; font-family:Consolas,Menlo,monospace;"
            "font-size:13px; background:transparent;")
        self.link_text = "starting"

    # -- commands ----------------------------------------------------------
    def command(self, channel: str, arg: str):
        cmd = f"{channel}:{arg}"
        self.pending[channel] = (cmd, time.time())
        self.link.send(cmd)
        self._last_status = cmd

    def drive(self, arg: str):
        if self.hook_deployed and arg in ("F", "B"):
            self._last_status = "BLOCKED - retract hook before driving"
            return
        self.command("DRV", arg)

    def toggle_hook(self):
        self.hook_deployed = not self.hook_deployed
        self.command("HOK", "D" if self.hook_deployed else "R")
        # interlock: no driving with the hook out
        self.btn_fwd.setEnabled(not self.hook_deployed)
        self.btn_back.setEnabled(not self.hook_deployed)
        if self.hook_deployed:
            self.command("DRV", "S")

    def on_led_changed(self, value: int):
        now = time.time()
        if now - self._last_led_send >= SLIDER_THROTTLE_MS / 1000.0:
            self._last_led_send = now
            self.command("LED", str(value))

    def on_led_released(self):
        self._last_led_send = 0.0
        self.command("LED", str(self.slider_led.value()))

    # -- keyboard ----------------------------------------------------------
    def keyPressEvent(self, e):
        if e.isAutoRepeat():
            return
        if e.key() == Qt.Key_Up:
            self.btn_fwd.setDown(True); self.drive("F")
        elif e.key() == Qt.Key_Down:
            self.btn_back.setDown(True); self.drive("B")
        elif e.key() == Qt.Key_Space:
            self.drive("S"); self.command("ACT", "S")
        elif e.key() == Qt.Key_Escape:
            self.close()

    def keyReleaseEvent(self, e):
        if e.isAutoRepeat():
            return
        if e.key() in (Qt.Key_Up, Qt.Key_Down):
            self.btn_fwd.setDown(False); self.btn_back.setDown(False)
            self.drive("S")

    # -- inbound -----------------------------------------------------------
    def on_frame(self, img: QImage):
        self.pane_left.show_frame(img)
        self.pane_right.show_frame(img)   # TODO: right pane = raw, left = annotated

    def on_telemetry(self, line: str):
        if line.startswith("ACK:"):
            cmd = line[4:]
            channel = cmd.split(":", 1)[0]
            self.acked[channel] = cmd
            self.pending.pop(channel, None)

    def set_link_text(self, text: str):
        self.link_text = text

    def refresh_status(self):
        now = time.time()
        stale = [c for c, (_, t) in self.pending.items()
                 if (now - t) * 1000 > ACK_TIMEOUT_MS]
        warn = f"   NO ACK: {','.join(stale)}" if stale else ""
        self.status.setText(
            f"{self.link_text}    {self.link.fps:5.1f} fps    "
            f"drive={self.acked.get('DRV', '-'):7s} "
            f"act={self.acked.get('ACT', '-'):7s} "
            f"hook={'OUT' if self.hook_deployed else 'IN':3s} "
            f"led={self.slider_led.value():3d}    {self._last_status}{warn}")

    def closeEvent(self, e):
        self.keepalive.stop()
        self.link.stop()
        super().closeEvent(e)


# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None, help="serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    args = ap.parse_args()

    if args.list:
        if not HAVE_SERIAL:
            print("pyserial not installed")
        else:
            for p in list_ports.comports():
                print(f"{p.device:20s} {p.description}")
        return 0

    app = QApplication(sys.argv)
    link = SerialLink(args.port, args.baud)
    panel = Panel(link)
    link.start()
    panel.show()
    panel.setFocus()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
