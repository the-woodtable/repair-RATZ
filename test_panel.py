#!/usr/bin/env python3
"""
test_panel.py — checks rat88_panel.py without needing a robot or a window.

    python3 test_panel.py

Two kinds of test in here:

1. LOGIC. The frame parser, the toggles, the button group, the odometer maths.
   These run headless; nothing is drawn.

2. FIRMWARE CROSS-CHECKS. The panel hard-codes command characters, servo
   angles and the actuator timeout, all of which live in "THIS ONE/src". These
   tests read the C++ and fail if the two drift apart — so if you change a
   `case` letter or an angle in main.cpp, this tells you instead of the robot
   quietly doing the wrong thing at the showcase.

No pytest needed. Exit code 0 = everything passed.
"""

import os
import re
import struct
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.chdir(HERE)

import rat88_panel as R          # noqa: E402
import auto_sequence as auto     # noqa: E402

FW = os.path.join(HERE, "THIS ONE", "src", "main.cpp")

_passed = []
_failed = []


def check(name):
    def deco(fn):
        try:
            fn()
        except AssertionError as exc:
            _failed.append((name, str(exc) or "assertion failed"))
        except Exception as exc:                      # noqa: BLE001
            _failed.append((name, f"{type(exc).__name__}: {exc}"))
        else:
            _passed.append(name)
        return fn
    return deco


# ---------------------------------------------------------------------------
# Test doubles
# ---------------------------------------------------------------------------

class FakeSignal:
    def __init__(self):
        self.slots = []

    def connect(self, fn):
        self.slots.append(fn)

    def fire(self, *a):
        for fn in self.slots:
            fn(*a)


class FakeButton:
    """Stands in for SelectableButton. `clicked` fires with a bool, exactly
    like QPushButton.clicked — that detail has caused a real bug before."""

    def __init__(self):
        self.sel = None
        self.clicked = FakeSignal()

    def set_selected(self, v):
        self.sel = v

    def is_selected(self):
        return bool(self.sel)

    def setDown(self, v):
        pass

    def click(self):
        self.clicked.fire(False)


class FakeLabel:
    def __init__(self):
        self.t = ""

    def setText(self, v):
        self.t = v


class FakeSlider:
    def __init__(self, v=9):
        self._v = v
        self.valueChanged = FakeSignal()

    def minimum(self):
        return 0

    def maximum(self):
        return 9

    def value(self):
        return self._v

    def setValue(self, v):
        if v != self._v:
            self._v = v
            self.valueChanged.fire(v)


class FakeKey:
    """A stand-in QKeyEvent. Most shortcuts are matched on text(), but the
    arrows and space are matched on key(), so both have to be available."""

    def __init__(self, text, key=None):
        self._t = text
        self._k = key if key is not None else object()

    def isAutoRepeat(self):
        return False

    def key(self):
        return self._k

    def text(self):
        return self._t


def make_panel():
    """A Panel with every widget replaced, so command logic can be exercised
    without a QApplication."""

    class P:
        hook_pos = R.HOOK_BOOT
        _last_cmd = ""
        act_until = 0.0
        act_dir = ""
        sent = []

        class link:
            @staticmethod
            def send(c):
                P.sent.append(c)

        send = R.Panel.send
        drive = R.Panel.drive
        stop_all = R.Panel.stop_all
        on_led = R.Panel.on_led
        toggle_led = R.Panel.toggle_led
        nudge_lamp = R.Panel.nudge_lamp
        on_lining = R.Panel.on_lining
        on_spool = R.Panel.on_spool
        on_hook = R.Panel.on_hook
        set_hook = R.Panel.set_hook
        on_telemetry = R.Panel.on_telemetry
        current_pos_mm = R.Panel.current_pos_mm
        auto_start = R.Panel.auto_start
        go_home = R.Panel.go_home
        on_run = R.Panel.on_run
        sync_run_buttons = R.Panel.sync_run_buttons
        zero_odometer = R.Panel.zero_odometer
        update_odometer = R.Panel.update_odometer
        keyPressEvent = R.Panel.keyPressEvent
        refresh_status = R.Panel.refresh_status

    p = P()
    P.sent = []
    p.led_on = R.LED_BOOT_ON
    p._led_last = None
    p._telem = {}
    p._telem_seen = 0.0
    p.odo = FakeLabel()
    p.slider = FakeSlider()
    p.slider.valueChanged.connect(p.on_led)
    p.btn_fwd, p.btn_back = FakeButton(), FakeButton()
    p.can_btn, p.spool_btn = FakeButton(), FakeButton()
    p.lining = R.Toggle(p.can_btn, R.LINING, p.on_lining)
    p.spool = R.Toggle(p.spool_btn, R.SPOOL, p.on_spool)
    p.hook = R.ButtonGroup(p.on_hook)
    p.hook_btns = {}
    for _, cmd, _, _ in R.HOOK_POSITIONS:
        p.hook_btns[cmd] = FakeButton()
        p.hook.add(cmd, p.hook_btns[cmd])
    p.hook.set_selected(p.hook_pos)
    p._dets = []
    p.auto = auto.AutoSequence(send=p.send, get_pos_mm=p.current_pos_mm)
    p.auto.inspection_dir = None
    p.start_btn, p.stop_btn = FakeButton(), FakeButton()
    p.run = R.ButtonGroup(p.on_run)
    p.run.add("start", p.start_btn)
    p.run.add("stop", p.stop_btn)
    p.run.set_selected("stop")
    return p


def frame(fid, body):
    return b"\xAA\x55" + bytes([fid]) + struct.pack("<I", len(body)) + body


JPEG = b"\xFF\xD8" + b"pretend image data" + b"\xFF\xD9"


# ---------------------------------------------------------------------------
# 1. Firmware cross-checks
# ---------------------------------------------------------------------------

@check("firmware: hook angles match HOOK_POSITIONS")
def _():
    fw = open(FW).read()
    for _, cmd, angle, _ in R.HOOK_POSITIONS:
        m = re.search(r"case '" + cmd + r"':\s*hookServo\.setAngle\((\d+)\)", fw)
        assert m, f"main.cpp has no hook case for '{cmd}'"
        assert int(m.group(1)) == angle, \
            f"'{cmd}': panel says {angle} deg, firmware says {m.group(1)}"
    boot = re.search(r"hookServo\.begin\(\w+,\s*(\d+)\)", fw)
    assert boot, "cannot find hookServo.begin"
    assert R.HOOK_ANGLE[R.HOOK_BOOT] == int(boot.group(1)), \
        f"HOOK_BOOT is {R.HOOK_BOOT} but the servo starts at {boot.group(1)} deg"


@check("firmware: spool commands and boot angle match SPOOL")
def _():
    fw = open(FW).read()
    on = re.search(r"case '" + R.SPOOL[1] + r"':\s*spoolServo\.setAngle\((\d+)\)", fw)
    off = re.search(r"case '" + R.SPOOL[0] + r"':\s*spoolServo\.setAngle\((\d+)\)", fw)
    assert on and off, "spool cases missing from main.cpp"
    boot = re.search(r"spoolServo\.begin\(\w+,\s*(\d+)\)", fw)
    assert boot, "cannot find spoolServo.begin"
    assert int(off.group(1)) == int(boot.group(1)), (
        f"SPOOL off state '{R.SPOOL[0]}' is {off.group(1)} deg but the servo "
        f"boots at {boot.group(1)} deg — the off artwork will be a lie")
    assert R.SPOOL[4] is False, "spool must start in its boot state"


@check("firmware: lining letters drive the actuator")
def _():
    fw = open(FW).read()
    for ch in (R.LINING[0], R.LINING[1]):
        assert re.search(r"case '" + ch + r"':\s*actuator\.", fw), \
            f"'{ch}' is not an actuator command in main.cpp"


@check("firmware: ACT_RUN_S matches Actuator::MAX_RUN_MS")
def _():
    hdr = open(os.path.join(HERE, "THIS ONE", "include", "Actuator.h")).read()
    m = re.search(r"MAX_RUN_MS\s*=\s*(\d+)", hdr)
    assert m, "cannot find MAX_RUN_MS"
    assert abs(R.ACT_RUN_S - int(m.group(1)) / 1000.0) < 0.01, \
        f"panel counts down {R.ACT_RUN_S}s, firmware cuts at {int(m.group(1))/1000}s"


@check("firmware: lamp boots off, and a digit implies on/off")
def _():
    led = open(os.path.join(HERE, "THIS ONE", "src", "LED.cpp")).read()
    assert re.search(r"_state\s*=\s*false", led)
    assert re.search(r"_bright\s*=\s*255", led)
    assert re.search(r"setBrightness\(uint8_t b\).*?_state\s*=\s*\(b\s*>\s*0\)",
                     led, re.S), "setBrightness no longer implies on/off"
    assert R.LED_BOOT_ON is False and R.LED_BOOT_LEVEL == 9
    fw = open(FW).read()
    assert re.search(r"case '" + R.LED_ON + r"':\s*led\.on\(\)", fw)
    assert re.search(r"case '" + R.LED_OFF + r"':\s*led\.off\(\)", fw)


@check("firmware: odometer comes from steps / STEPS_PER_MM, zeroed by Z")
def _():
    fw = open(FW).read()
    assert re.search(r"getStepCount\(\)\s*/\s*STEPS_PER_MM", fw), \
        "pos_mm is no longer derived from the step count"
    assert R.ODO_FIELD in fw, f"{R.ODO_FIELD} is not in sendTelemetry"
    assert re.search(r"case '" + R.ODO_CMD_ZERO + r"':\s*stepper\.zero\(\)", fw)
    body = fw[fw.index("void sendTelemetry"):fw.index("void loop")]
    assert "0xAA" not in body and ".write(" not in body, \
        "telemetry is framed now — SerialLink._stash_text is no longer needed"


@check("firmware: drive letters still F/B/S with no timeout")
def _():
    fw = open(FW).read()
    for ch, arg in (("F", r"\+1"), ("B", "-1"), ("S", "0")):
        assert re.search(r"case '" + ch + r"':\s*stepper\.setDrive\(" + arg + r"\)", fw)
    stepper = open(os.path.join(HERE, "THIS ONE", "src", "stepper.cpp")).read()
    assert "MAX_RUN" not in stepper, \
        "the stepper grew a timeout — the arrows could become click-once"


# ---------------------------------------------------------------------------
# 2. Serial protocol
# ---------------------------------------------------------------------------

@check("parser: camera ids, telemetry frames, sensor padding")
def _():
    link = R.SerialLink(None)
    cams, telem = [], []
    link.frame_ready.connect(lambda c, b: cams.append(c))
    link.telemetry.connect(lambda s: telem.append(s))
    link._buf = bytearray(
        frame(0x00, JPEG + b"PAD") + frame(0x01, JPEG)
        + frame(0x4C, JPEG) + frame(0x52, JPEG)
        + frame(0x02, b'{"pos_mm":12.0}'))
    link._parse()
    assert cams == ["L", "R", "L", "R"], cams
    assert telem == ['{"pos_mm":12.0}'], telem


@check("parser: raw telemetry between frames is not discarded")
def _():
    # sendTelemetry() uses plain Serial.print, so its text arrives OUTSIDE the
    # frame protocol. Dropping non-magic bytes loses every reading.
    link = R.SerialLink(None)
    cams, telem = [], []
    link.frame_ready.connect(lambda c, b: cams.append(c))
    link.telemetry.connect(lambda s: telem.append(s))
    line = b'{"steps":1234{"pos_mm":383.0{"led":255}\r\n'
    link._buf = bytearray(frame(0x00, JPEG) + line + frame(0x01, JPEG))
    link._parse()
    assert cams == ["L", "R"], cams
    assert len(telem) == 1 and "pos_mm" in telem[0], telem


@check("parser: survives byte-at-a-time arrival")
def _():
    link = R.SerialLink(None)
    cams, telem = [], []
    link.frame_ready.connect(lambda c, b: cams.append(c))
    link.telemetry.connect(lambda s: telem.append(s))
    blob = frame(0x00, JPEG) + b'{"pos_mm":7.5}\n' + frame(0x01, JPEG)
    for byte in blob:
        link._buf.append(byte)
        link._parse()
    assert cams == ["L", "R"], cams
    assert len(telem) == 1 and "pos_mm" in telem[0], telem


@check("parser: resync past garbage, bogus lengths, missing JPEG header")
def _():
    link = R.SerialLink(None)
    cams = []
    link.frame_ready.connect(lambda c, b: cams.append(c))
    link._buf = bytearray(b"\x00\xAA\x11junk" + frame(0x01, JPEG))
    link._parse()
    assert cams == ["R"], cams

    cams.clear()
    link._buf = bytearray(b"\xAA\x55\x00" + struct.pack("<I", 9_000_000)
                          + frame(0x00, JPEG))
    link._parse()
    assert cams == ["L"], cams

    cams.clear()
    link._buf = bytearray(frame(0x00, b"\x00\x01 no start marker \xFF\xD9"))
    link._parse()
    assert cams == [], "a frame without FFD8 must be dropped"


@check("parser: loose-text buffer cannot grow without bound")
def _():
    link = R.SerialLink(None)
    link._stash_text(b"\x01\x02" * 8000)          # binary junk, never a newline
    assert len(link._textbuf) <= 4096, len(link._textbuf)


# ---------------------------------------------------------------------------
# 3. Controls
# ---------------------------------------------------------------------------

@check("group: exactly one hook position lit, and Qt's bool is ignored")
def _():
    p = make_panel()
    for _, cmd, _, _ in R.HOOK_POSITIONS:
        p.hook_btns[cmd].click()
        lit = [c for c, b in p.hook_btns.items() if b.is_selected()]
        assert lit == [cmd], f"clicking {cmd} left {lit} lit"
        assert p.sent[-1] == cmd, f"clicking {cmd} sent {p.sent[-1]!r}"
        assert p.hook_pos == cmd


@check("toggle: lining and spool alternate and stay independent")
def _():
    p = make_panel()
    p.can_btn.click()
    assert p.sent == ["D"] and p.can_btn.sel is True and p.spool_btn.sel is False
    p.spool_btn.click()
    assert p.sent == ["D", "T"] and p.can_btn.sel is True, "spool disturbed lining"
    p.can_btn.click()
    assert p.sent == ["D", "T", "R"] and p.spool_btn.sel is True, \
        "lining disturbed spool"


@check("toggle: only the actuator arms a countdown")
def _():
    p = make_panel()
    p.spool_btn.click()
    assert p.act_until == 0.0, "a servo must not start a motor countdown"
    p.can_btn.click()
    assert p.act_until > time.time() and p.act_dir == "deploying"


@check("stop_all: cuts the motor without lying about positions")
def _():
    p = make_panel()
    p.can_btn.click()
    p.spool_btn.click()
    can_before, spool_before = p.can_btn.sel, p.spool_btn.sel
    p.stop_all()
    assert p.sent[-2:] == ["S", "X"], p.sent
    assert p.act_until == 0.0 and p.act_dir == "stopped"
    assert p.can_btn.sel is can_before, "stop_all moved the lining artwork"
    assert p.spool_btn.sel is spool_before, "stop_all disturbed the servo"


@check("keyboard: keys move the artwork, not just the wire")
def _():
    p = make_panel()
    p.keyPressEvent(FakeKey("d"))
    assert p.sent[-1] == "D" and p.can_btn.sel is True, "key did not light the can"
    p.keyPressEvent(FakeKey("r"))
    assert p.sent[-1] == "R" and p.can_btn.sel is False
    p.keyPressEvent(FakeKey("t"))
    assert p.sent[-1] == "T" and p.spool_btn.sel is True
    for _, cmd, _, _ in R.HOOK_POSITIONS:
        p.keyPressEvent(FakeKey(cmd.lower()))
        lit = [c for c, b in p.hook_btns.items() if b.is_selected()]
        assert lit == [cmd], f"key {cmd} lit {lit}"


@check("keyboard: mouse and keyboard stay in phase")
def _():
    p = make_panel()
    p.can_btn.click()                       # -> deployed
    p.keyPressEvent(FakeKey("r"))           # -> compressed
    assert p.can_btn.sel is False
    p.can_btn.click()                       # must deploy again, not retract
    assert p.can_btn.sel is True and p.sent[-1] == "D"


@check("lamp: L toggles, =/- step and clamp, digits imply on/off")
def _():
    p = make_panel()
    assert p.led_on is False
    p.keyPressEvent(FakeKey("l"))
    assert p.sent[-1] == "L" and p.led_on is True
    p.keyPressEvent(FakeKey("L"))
    assert p.sent[-1] == "O" and p.led_on is False

    p.keyPressEvent(FakeKey("-"))
    assert p.slider.value() == 8 and p.sent[-1] == "8"
    assert p.led_on is True, "any digit above 0 lights the lamp in firmware"
    p.keyPressEvent(FakeKey("="))
    assert p.slider.value() == 9
    p.keyPressEvent(FakeKey("="))
    assert p.slider.value() == 9, "must clamp at maximum"
    for _ in range(9):
        p.keyPressEvent(FakeKey("-"))
    assert p.slider.value() == 0 and p.led_on is False, "'0' turns the lamp off"
    p.keyPressEvent(FakeKey("-"))
    assert p.slider.value() == 0, "must clamp at minimum"


@check("keyboard: no two controls share a letter")
def _():
    keys = (set(R.HOOK_LABEL)
            | {R.LINING[0], R.LINING[1], R.SPOOL[0], R.SPOOL[1],
               R.LED_ON, R.ODO_CMD_ZERO})
    assert len(keys) == 9, sorted(keys)
    assert not (keys & {"S", "F", "B", "X", R.LED_OFF}), \
        f"a shortcut collides with a reserved command: {sorted(keys)}"


# ---------------------------------------------------------------------------
# 4. Odometer
# ---------------------------------------------------------------------------

@check("odometer: same number control_panel_stereo2 shows")
def _():
    # That panel does  DISTANCE TRAVELED: {t['pos_mm'] / 10:.1f} cm
    p = make_panel()
    p.update_odometer()
    assert p.odo.t == "--", "nothing received yet should read --"

    p.on_telemetry('{"steps":1234{"pos_mm":383.0{"led":255}')
    p.update_odometer()
    assert p.odo.t == "38.3", f"383.0 mm should be 38.3 cm, got {p.odo.t}"

    p.on_telemetry('{"pos_mm":-125.5}')
    p.update_odometer()
    assert p.odo.t.startswith("-12."), f"reversing should read negative: {p.odo.t}"

    p.sent.clear()
    p.zero_odometer()
    assert p.sent == [R.ODO_CMD_ZERO] and p.odo.t == "0.0"
    assert R.ODO_FIELD not in p._telem


@check("odometer: falls back to raw steps, like the old panel")
def _():
    p = make_panel()
    p.on_telemetry('{"steps":1288{"led":255}')       # firmware without pos_mm
    p.update_odometer()
    expected = 1288 / R.STEPS_PER_MM / 10.0
    assert p.odo.t == f"{expected:.1f}", \
        f"steps fallback gave {p.odo.t}, expected {expected:.1f}"


@check("odometer: STEPS_PER_MM fallback matches the firmware constant")
def _():
    fw = open(FW).read()
    m = re.search(r"STEPS_PER_MM\s*=\s*([\d.]+)f?", fw)
    assert m, "cannot find STEPS_PER_MM in main.cpp"
    assert abs(R.STEPS_PER_MM - float(m.group(1))) < 1e-6, \
        f"panel has {R.STEPS_PER_MM}, firmware has {m.group(1)}"


@check("odometer: a garbled value cannot blank a good reading")
def _():
    p = make_panel()
    p.on_telemetry('{"pos_mm":383.0}')
    p.update_odometer()
    good = p.odo.t
    p._telem["pos_mm"] = "not-a-number"
    p.update_odometer()
    assert p.odo.t == good, "one bad packet wiped the readout"


@check("odometer: staleness blanking works when switched on")
def _():
    p = make_panel()
    p.on_telemetry('{"pos_mm":383.0}')
    p.update_odometer()
    assert p.odo.t == "38.3"
    saved = R.ODO_STALE_S
    try:
        R.ODO_STALE_S = 3.0
        p._telem_seen = time.time() - 10
        p.update_odometer()
        assert p.odo.t == "--", "stale reading was not blanked"
        p._telem_seen = time.time()
        p.update_odometer()
        assert p.odo.t == "38.3", "fresh reading did not come back"
    finally:
        R.ODO_STALE_S = saved


@check("odometer: the tick actually calls it")
def _():
    # This is the bug that made the readout sit at "--" forever: the method
    # existed and was correct, but nothing ever called it. Testing the maths
    # in isolation could not catch that, so drive the real tick instead.
    p = make_panel()
    p.status = FakeLabel()
    p.link_text = "SIM"
    p.link.fps = {"L": 0.0, "R": 0.0}
    p.cv = None
    p.cv_note = ""
    p.auto_status = "idle"
    p._det_count = 0
    p._nearest = None
    p.on_telemetry('{"pos_mm":456.0}')
    assert p.odo.t in ("", "--"), "precondition: readout not yet written"
    R.Panel.refresh_status(p)
    assert p.odo.t == "45.6", \
        f"refresh_status did not update the odometer (got {p.odo.t!r})"


@check("odometer: still updates with the status strip hidden")
def _():
    p = make_panel()
    p.status = FakeLabel()
    p.link_text = "SIM"
    p.link.fps = {"L": 0.0, "R": 0.0}
    p.cv = None
    p.cv_note = ""
    p.auto_status = "idle"
    p._det_count = 0
    p._nearest = None
    p.on_telemetry('{"pos_mm":789.0}')
    saved = R.SHOW_STATUS
    try:
        R.SHOW_STATUS = False
        R.Panel.refresh_status(p)
        assert p.odo.t == "78.9", \
            "the odometer stopped updating when the debug strip was hidden"
    finally:
        R.SHOW_STATUS = saved


@check("odometer: Z key and clicking the readout do the same thing")
def _():
    p = make_panel()
    p.keyPressEvent(FakeKey("z"))
    assert p.sent[-1] == R.ODO_CMD_ZERO and p.odo.t == "0.0"


# ---------------------------------------------------------------------------
# 4b. The autonomous sequence
# ---------------------------------------------------------------------------

class Det:
    def __init__(self, conf, track_id=1, label="crack"):
        self.conf = conf
        self.track_id = track_id
        self.label = label


def run_auto(pos_mm=0.0):
    """An AutoSequence wired to a fake robot: records what it sends, and
    reports whatever position the test sets."""
    sent = []
    box = {"pos": pos_mm}
    a = auto.AutoSequence(send=sent.append, get_pos_mm=lambda: box["pos"])
    a.inspection_dir = None
    return a, sent, box


@check("auto: constants have not drifted from control_panel_stereo2")
def _():
    old = open(os.path.join(HERE, "control_panel_stereo2.py")).read()
    for name in ("AUTO_DEPLOY_CONF", "AUTO_PAUSE_TRIGGER_CONF",
                 "AUTO_REPOSITION_MM", "PRE_DEPLOY_WAIT_SECS",
                 "POST_DEPLOY_WAIT_SECS", "ACTUATOR_RUN_SECS",
                 "HOME_TOLERANCE_MM", "AUTO_PAUSE_SETTLE_TICKS",
                 "AUTO_PAUSE_CONFIRM_TICKS", "SCREENSHOT_CONF_MIN",
                 "SCREENSHOT_CONF_MAX"):
        m = re.search(name + r"\s*=\s*([\d.]+)", old)
        assert m, f"{name} is gone from control_panel_stereo2.py"
        mine = getattr(auto, name)
        assert abs(float(m.group(1)) - float(mine)) < 1e-9, \
            f"{name}: auto_sequence has {mine}, the old panel has {m.group(1)}"


@check("auto: start drives forward, stop cuts drive AND actuator")
def _():
    a, sent, _ = run_auto()
    assert a.state == "IDLE"
    a.start()
    assert a.state == "SCANNING" and sent == ["F"], sent
    sent.clear()
    a.stop()
    assert a.state == "IDLE"
    assert "S" in sent and "X" in sent, \
        f"stop must halt the drive and the actuator, sent {sent}"


@check("auto: a weak detection is ignored, a strong one stops the robot")
def _():
    a, sent, _ = run_auto()
    a.start()
    sent.clear()
    a.step([Det(auto.AUTO_PAUSE_TRIGGER_CONF - 0.01)])
    assert a.state == "SCANNING", "a below-threshold detection stopped the scan"
    a.step([Det(auto.AUTO_PAUSE_TRIGGER_CONF + 0.01)])
    assert a.state == "PAUSED" and sent[-1] == "S", sent


def settle_and_confirm(a, conf, t=1000.0):
    """Drive a PAUSED sequence through settling and confirmation, stopping on
    the tick the decision is made."""
    for _i in range(auto.AUTO_PAUSE_SETTLE_TICKS
                    + auto.AUTO_PAUSE_CONFIRM_TICKS):
        a.step([Det(conf)], t)


@check("auto: low average confidence resumes scanning, high one commits")
def _():
    for avg_conf, expect in ((0.45, "SCANNING"), (0.95, "REPOSITIONING")):
        a, sent, _ = run_auto()
        a.start()
        a.step([Det(0.9)])                       # -> PAUSED
        settle_and_confirm(a, avg_conf)
        assert a.state == expect, \
            f"average {avg_conf} should give {expect}, got {a.state}"


@check("auto: a mid-band crack re-pauses instead of scanning past it")
def _():
    # Between AUTO_PAUSE_TRIGGER_CONF and AUTO_DEPLOY_CONF the sequence gives
    # up on deploying, resumes scanning, and then immediately trips its own
    # trigger again on the same crack. Inherited from control_panel_stereo2.
    # Pinned here so the behaviour is a decision, not a surprise on the day.
    mid = (auto.AUTO_PAUSE_TRIGGER_CONF + auto.AUTO_DEPLOY_CONF) / 2
    a, _s, _b = run_auto()
    a.start()
    a.step([Det(0.9)])
    settle_and_confirm(a, mid)
    assert a.state == "SCANNING", a.state
    a.step([Det(mid)])
    assert a.state == "PAUSED", \
        "expected the same crack to re-trigger; behaviour has changed"


@check("auto: settling ticks are not counted as confidence samples")
def _():
    a, _s, _ = run_auto()
    a.start()
    a.step([Det(0.9)])                           # -> PAUSED
    for _i in range(auto.AUTO_PAUSE_SETTLE_TICKS):
        a.step([Det(0.0)])                       # blurred frames, ignored
    assert a._confs == [], "settling frames were sampled"
    a.step([Det(0.9)])
    assert a._confs == [0.9]


@check("auto: full sequence reaches IDLE and leaves nothing running")
def _():
    a, sent, box = run_auto()
    t = 1000.0
    a.start()
    a.step([Det(0.9)], t)
    settle_and_confirm(a, 0.95, t)
    assert a.state == "REPOSITIONING"
    box["pos"] = auto.AUTO_REPOSITION_MM + 1     # far enough past the crack
    a.step([], t)
    assert a.state == "PRE_DEPLOY_WAIT" and sent[-1] == "S"
    t += auto.PRE_DEPLOY_WAIT_SECS + 0.1
    a.step([], t)
    assert a.state == "DEPLOYING" and sent[-1] == "D"
    t += auto.ACTUATOR_RUN_SECS + 0.1
    a.step([], t)
    assert a.state == "POST_DEPLOY_WAIT" and sent[-1] == "X"
    t += auto.POST_DEPLOY_WAIT_SECS + 0.1
    a.step([], t)
    assert a.state == "RETRACTING_AUTO" and sent[-1] == "R"
    t += auto.ACTUATOR_RUN_SECS + 0.1
    a.step([], t)
    assert a.state == "IDLE" and sent[-1] == "X", sent


@check("auto: repositioning without telemetry stops instead of driving blind")
def _():
    a, sent, box = run_auto()
    a.state = "REPOSITIONING"
    a._reposition_start_mm = 0.0
    a.drive_state = 1
    box["pos"] = None
    a.step([], 1000.0)
    assert a.state == "PRE_DEPLOY_WAIT", a.state
    assert sent[-1] == "S", "must stop when position feedback is lost"


@check("auto: home drives the right way, arrives, then zeroes")
def _():
    for start_pos, first_cmd in ((120.0, "B"), (-120.0, "F")):
        a, sent, box = run_auto(start_pos)
        assert a.return_home(zero_on_arrival=True) is True
        a.step([], 1000.0)
        assert sent[-1] == first_cmd, \
            f"from {start_pos}mm the first move should be {first_cmd}, got {sent}"
        box["pos"] = 0.0                          # arrived
        a.step([], 1000.1)
        assert a.state == "IDLE"
        assert "S" in sent and sent[-1] == "Z", \
            f"must stop then zero on arrival, sent {sent}"


@check("auto: home refuses without telemetry or while busy")
def _():
    a, _s, box = run_auto()
    box["pos"] = None
    assert a.return_home() is False and a.state == "IDLE"
    a2, _s2, _b2 = run_auto()
    a2.start()
    assert a2.return_home() is False, "homing must not interrupt a scan"
    assert a2.state == "SCANNING"


@check("auto: homing ignores cracks entirely")
def _():
    a, sent, box = run_auto(200.0)
    a.return_home()
    sent.clear()
    a.step([Det(0.99)], 1000.0)                   # a very confident crack
    assert a.state == "RETURNING_HOME", "homing stopped for a detection"
    assert sent[-1] == "B"


@check("auto: screenshots only in the review band, once per crack, when active")
def _():
    a, _s, _b = run_auto()
    band = (auto.SCREENSHOT_CONF_MIN + auto.SCREENSHOT_CONF_MAX) / 2
    a.state = "IDLE"
    assert a.maybe_capture(None, Det(band), "L") is None, \
        "captured while idle"
    a.state = "SCANNING"
    assert a.maybe_capture(None, Det(auto.SCREENSHOT_CONF_MAX + 0.1), "L") is None, \
        "captured a high-confidence crack the robot acts on automatically"
    assert a.maybe_capture(None, Det(auto.SCREENSHOT_CONF_MIN - 0.1), "L") is None, \
        "captured a detection too weak to be worth reviewing"
    assert a.state in auto.SCREENSHOT_ACTIVE_STATES
    for st in ("POST_DEPLOY_WAIT", "RETRACTING_AUTO", "RETURNING_HOME"):
        assert st not in auto.SCREENSHOT_ACTIVE_STATES, \
            f"{st} would screenshot with the lining already out"


@check("panel: STOP cancels the sequence before sending S")
def _():
    p = make_panel()
    p.auto.start()
    assert p.auto.running
    p.sent.clear()
    p.stop_all()
    assert not p.auto.running, "STOP left the sequence running — it would resume"
    assert "S" in p.sent and "X" in p.sent, p.sent


@check("run pair: exactly one selected, and it is the greyed one")
def _():
    p = make_panel()
    assert p.run.selected() == "stop", "STOP should be selected before anything runs"
    p.start_btn.click()
    assert p.run.selected() == "start" and p.start_btn.sel is True
    assert p.stop_btn.sel is False, "both buttons cannot be selected at once"
    p.stop_btn.click()
    assert p.run.selected() == "stop" and p.stop_btn.sel is True
    assert p.start_btn.sel is False


@check("run pair: clicking start actually starts, clicking stop actually stops")
def _():
    p = make_panel()
    p.start_btn.click()
    assert p.auto.running and p.auto.state == "SCANNING", p.auto.state
    p.sent.clear()
    p.stop_btn.click()
    assert not p.auto.running
    assert "S" in p.sent and "X" in p.sent, p.sent


@check("run pair: follows the sequence when it ends by itself")
def _():
    p = make_panel()
    p.start_btn.click()
    assert p.run.selected() == "start"
    # The sequence finishes on its own - nothing goes through the buttons.
    p.auto.state = "IDLE"
    p.sync_run_buttons()
    assert p.run.selected() == "stop", \
        "buttons still claimed to be running after the sequence ended"


@check("run pair: space bar and lost focus also move the buttons back")
def _():
    p = make_panel()
    p.start_btn.click()
    p.keyPressEvent(FakeKey(" ", key=R.Qt.Key_Space))   # space = stop_all
    p.sync_run_buttons()
    assert not p.auto.running and p.run.selected() == "stop"


@check("run pair: syncing must not re-issue the command")
def _():
    p = make_panel()
    p.start_btn.click()
    p.sent.clear()
    for _i in range(5):
        p.sync_run_buttons()               # already correct, must be silent
    assert p.sent == [], f"sync re-sent commands: {p.sent}"


@check("panel: HOME goes through the sequence, not straight to Z")
def _():
    p = make_panel()
    p.on_telemetry('{"pos_mm":150.0}')
    p.sent.clear()
    p.go_home()
    assert p.auto.state == "RETURNING_HOME", p.auto.state
    assert "Z" not in p.sent, "HOME zeroed before the robot had moved"


# ---------------------------------------------------------------------------
# 5. Layout and artwork
# ---------------------------------------------------------------------------

DECOR = {"header", "mouse_top", "mouse_bottom"}   # deliberately bleed off-canvas


def _rect(key):
    a = R.GEO[key]
    return (a.x(), a.y(), a.x() + a.width(), a.y() + a.height())


@check("assets: every configured file exists")
def _():
    missing = R.check_assets()
    assert not missing, "\n" + "\n".join(missing)


@check("assets: no artwork is distorted by its box")
def _():
    try:
        from PIL import Image
    except ImportError:
        return                                    # optional dependency
    worst = (0, None)
    for key in ("arrow_up", "arrow_down", "can", "bezel", "header",
                "mouse_top", "mouse_bottom", "distance",
                "start", "stop", "home", "folder",
                *[n for n, _, _, _ in R.HOOK_POSITIONS]):
        im = Image.open(R.ASSETS[key])
        a = R.GEO[key]
        art = im.size[0] / im.size[1]
        box = a.width() / a.height()
        slack = abs(art - box) / box
        if slack > worst[0]:
            worst = (slack, key)
    assert worst[0] < 0.05, \
        f"GEO['{worst[1]}'] is {worst[0]*100:.0f}% the wrong shape for its art"


@check("layout: controls on canvas, no overlaps, panes inside the bezel")
def _():
    for key in R.GEO:
        if key in DECOR:
            continue
        x0, y0, x1, y1 = _rect(key)
        assert 0 <= x0 and 0 <= y0 and x1 <= 1280 and y1 <= 960, (key, _rect(key))

    bx0, by0, bx1, by1 = _rect("bezel")
    for key in ("video_left", "video_right"):
        x0, y0, x1, y1 = _rect(key)
        assert bx0 <= x0 and by0 <= y0 and x1 <= bx1 and y1 <= by1, \
            f"{key} is not inside the bezel"
    assert _rect("video_left")[2] <= _rect("video_right")[0], "panes overlap"

    clickable = ["can", "distance", "arrow_up", "arrow_down",
                 "start", "stop", "home", "folder",
                 *[n for n, _, _, _ in R.HOOK_POSITIONS]]
    for i, a in enumerate(clickable):
        for b in clickable[i + 1:]:
            ra, rb = _rect(a), _rect(b)
            overlap = not (ra[2] <= rb[0] or rb[2] <= ra[0]
                           or ra[3] <= rb[1] or rb[3] <= ra[1])
            assert not overlap, f"{a} overlaps {b}: {ra} vs {rb}"

    bottom = max(_rect(k)[3] for k in ("can", "folder", "home")
                 if k in R.GEO)
    assert _rect("status")[1] >= bottom, "status strip covers the button row"


@check("scaling: uniform, centred, and nothing falls off the screen")
def _():
    class Fake:
        pass

    for w, h in ((1280, 960), (1470, 956), (1920, 1080), (2560, 800), (800, 2000)):
        p = Fake()
        p.S = min(w / 1280, h / 960)
        p.ox = int((w - 1280 * p.S) / 2)
        p.oy = int((h - 960 * p.S) / 2)
        p.g = R.Panel.g.__get__(p)
        p.px = R.Panel.px.__get__(p)
        assert abs(p.S - min(w / 1280, h / 960)) < 1e-9, "scale is not uniform"
        assert p.px(2) >= 1 and p.px(11) >= 1, "px() rounded down to zero"
        for key in R.GEO:
            if key in DECOR:
                continue
            r = p.g(key)
            assert r.x() >= 0 and r.y() >= 0, (w, h, key)
            assert r.x() + r.width() <= w + 1, (w, h, key)
            assert r.y() + r.height() <= h + 1, (w, h, key)


@check("safety: closeEvent exists and stops the link")
def _():
    src = open(os.path.join(HERE, "rat88_panel.py")).read()
    assert "\n    def closeEvent(self, e):" in src, \
        "closeEvent is missing or commented out — motors keep running on exit"
    body = src.split("def closeEvent")[1][:400]
    assert "self.link.stop()" in body, "closeEvent no longer stops the link"
    assert "def send(self" in src


# ---------------------------------------------------------------------------

def main():
    for name in _passed:
        print(f"  ok    {name}")
    for name, why in _failed:
        print(f"  FAIL  {name}\n          {why}")
    print(f"\n{len(_passed)} passed, {len(_failed)} failed")
    return 1 if _failed else 0


if __name__ == "__main__":
    sys.exit(main())
