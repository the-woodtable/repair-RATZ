"""
auto_sequence.py — the autonomous scan/deploy sequence, without any GUI.

Ported from control_panel_stereo2.py so rat88_panel.py can run the same
behaviour. It is a COPY, not a shared import: control_panel_stereo2.py builds
a Tk application at import time, so importing it from a Qt panel would be a
mess. The cost of that choice is that the constants and the state machine now
exist in two places.

    >>> IF YOU RETUNE ANYTHING IN control_panel_stereo2.py, CHANGE IT HERE TOO.
    >>> test_panel.py compares the constants in both files and fails if they
    >>> drift apart, so you will be told rather than surprised.

Nothing in here touches the serial port, the screen, or OpenCV windows. It
talks to the outside world through three callbacks handed in by the panel:

    send(ch)        put one command character on the wire
    get_pos_mm()    latest odometer reading in mm, or None if no telemetry
    on_status(txt)  optional; a line of human-readable progress

Call step(detections, now) once per UI tick. `detections` is this frame's
confirmed-crack list; each item needs .conf, .track_id and .label.
"""

import os
import re
import time

try:
    import cv2
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False


# ---------------------------------------------------------------------------
# Tuning. Every one of these is copied from control_panel_stereo2.py.
# ---------------------------------------------------------------------------

# A detection needs this much confidence to make the robot stop and look.
# Below it, scanning just carries on.
AUTO_PAUSE_TRIGGER_CONF = 0.5
# ...and this much, averaged, to actually commit to deploying a lining. Much
# higher on purpose: a false positive here wastes travel and operator time.
AUTO_DEPLOY_CONF = 0.60
# How far to drive PAST a confirmed crack before deploying, in mm.
AUTO_REPOSITION_MM = 30.0
PRE_DEPLOY_WAIT_SECS = 5.0
POST_DEPLOY_WAIT_SECS = 5.0
# The actuator has no position feedback and no limit switch - telemetry's
# "actuator" field is a mode flag that goes still the instant motion starts.
# Completion cannot be detected, only timed.
#
# This MUST NOT exceed Actuator::MAX_RUN_MS in THIS ONE/include/Actuator.h
# (10000ms), because that is what actually cuts the motor. Waiting longer does
# not run the actuator for longer - it just leaves the sequence believing it
# is still deploying for seconds after the motor already stopped, and the
# panel counting down to a moment that has passed.
#
# Deploy and retract were both measured at 10s, so this matches the firmware
# cutoff exactly. If you need real margin, raise MAX_RUN_MS first, then this.
ACTUATOR_RUN_SECS = 10.0
# Close enough to home to call it arrived, in mm.
HOME_TOLERANCE_MM = 2.0
# Ticks to wait after stopping before trusting a confidence reading - the
# first few frames after 'S' are still motion-blurred from driving.
AUTO_PAUSE_SETTLE_TICKS = 5
# Then average over this many ticks. A single frame can spike or drop from
# lighting flicker or a partial view.
AUTO_PAUSE_CONFIRM_TICKS = 15

# Ambiguous cracks - confident enough to be worth a human look, not confident
# enough to act on - get one screenshot each.
SCREENSHOT_CONF_MIN = 0.4
SCREENSHOT_CONF_MAX = 0.79
# Only while a scan is actually running, and only before the lining goes out.
SCREENSHOT_ACTIVE_STATES = frozenset(
    {"SCANNING", "PAUSED", "REPOSITIONING", "PRE_DEPLOY_WAIT", "DEPLOYING"})

DATA_DIR = os.path.expanduser("~/Desktop/30.007/pipe_cam_data")

# States the sequence can be in. IDLE means the panel has manual control.
STATES = ("IDLE", "SCANNING", "PAUSED", "REPOSITIONING", "PRE_DEPLOY_WAIT",
          "DEPLOYING", "POST_DEPLOY_WAIT", "RETRACTING_AUTO", "RETURNING_HOME")


def next_numbered_dir(base_dir, prefix):
    """base_dir/"{prefix} (N)" for the smallest unused N, so each run gets a
    fresh folder instead of mixing with the last one."""
    os.makedirs(base_dir, exist_ok=True)
    pattern = re.compile(re.escape(prefix) + r" \((\d+)\)$")
    used = [int(m.group(1)) for m in
            (pattern.match(n) for n in os.listdir(base_dir)) if m]
    path = os.path.join(base_dir, f"{prefix} ({max(used, default=0) + 1})")
    os.makedirs(path, exist_ok=True)
    return path


class AutoSequence:
    """The scan -> confirm -> reposition -> deploy -> retract state machine,
    plus the separate RETURNING_HOME path.

    Safety notes carried over from the original:
      * Every state that drives sends 'S' before leaving, so cancelling at any
        point stops the motor.
      * RETURNING_HOME ignores detections completely. Once homing starts it
        must not stop for a crack.
      * Losing telemetry mid-move fails to a stop rather than driving blind.
    """

    def __init__(self, send, get_pos_mm, on_status=None, inspection_dir=None):
        self.send = send
        self.get_pos_mm = get_pos_mm
        self.on_status = on_status or (lambda _t: None)

        self.state = "IDLE"
        self.status = "idle"
        self.drive_state = 0            # -1 back, 0 stopped, +1 forward

        self._confs = []
        self._pause_ticks = 0
        # Per-instance copy of the deploy bar so the panel's C key can change
        # it at runtime. AUTO_DEPLOY_CONF is only the starting value.
        self.deploy_conf = AUTO_DEPLOY_CONF
        self._reposition_start_mm = None
        self._wait_started_t = 0.0
        self._act_started_t = 0.0
        self._zero_on_arrival = False

        self.inspection_dir = inspection_dir
        self.shots = []                  # paths saved this run, newest last
        self._shot_ids = set()           # (tag, track_id) already captured

    # -- helpers -----------------------------------------------------------
    def _say(self, text):
        self.status = text
        self.on_status(text)

    def _drive(self, d):
        """d: -1 back, 0 stop, +1 forward. Only sends on change."""
        if self.drive_state != d:
            self.send({-1: "B", 0: "S", 1: "F"}[d])
            self.drive_state = d

    @property
    def running(self):
        return self.state != "IDLE"

    def set_deploy_conf(self, value: float):
        """Change the bar for committing to a lining, while running.

        This is NOT the detection threshold. YOLO still reports everything
        above CV_CONF (0.4) and the panel still draws it; this only moves the
        point at which the robot decides a crack is worth acting on.
        """
        self.deploy_conf = float(value)

    def ensure_dir(self):
        if self.inspection_dir is None:
            self.inspection_dir = next_numbered_dir(DATA_DIR, "Pipe Inspection")
        return self.inspection_dir

    # -- entry points ------------------------------------------------------
    def start(self):
        """START button. Begins scanning forward."""
        if self.running:
            return False
        self.ensure_dir()
        self.state = "SCANNING"
        self._confs = []
        self._pause_ticks = 0
        self._drive(1)
        self._say("scanning")
        return True

    def stop(self):
        """STOP button. Cancels whatever is running and stops everything.

        Sends 'X' as well as 'S' because this may have cancelled mid deploy or
        mid retract, and the actuator would otherwise keep running to its own
        10s timeout.
        """
        was = self.state
        self.state = "IDLE"
        self._zero_on_arrival = False
        self._drive(0)
        self.send("X")
        self._say("stopped" if was != "IDLE" else "idle")
        return was

    def return_home(self, zero_on_arrival=True):
        """HOME button. Drives back to position 0, then zeroes the odometer.

        Refuses if a crack sequence is running - that must finish or be
        stopped first - and refuses without telemetry, since there is no way
        to know when it has arrived.
        """
        if self.running:
            self._say("busy — stop the sequence before homing")
            return False
        if self.get_pos_mm() is None:
            self._say("no telemetry — cannot home")
            return False
        self.state = "RETURNING_HOME"
        self._zero_on_arrival = zero_on_arrival
        self._say("returning home")
        return True

    # -- the tick ----------------------------------------------------------
    def step(self, detections, now=None):
        """Advance one tick. Safe to call at any rate; the waits are wall
        clock, only the PAUSED confirmation counts ticks."""
        now = time.time() if now is None else now
        handler = getattr(self, "_st_" + self.state.lower(), None)
        if handler is not None:
            handler(detections, now)

    def _st_idle(self, detections, now):
        pass

    def _st_scanning(self, detections, now):
        self._drive(1)
        if any(d.conf >= AUTO_PAUSE_TRIGGER_CONF for d in detections):
            # Stop first, then look. A moving frame is motion-blurred and its
            # confidence number is not worth acting on.
            self._drive(0)
            self.state = "PAUSED"
            self._pause_ticks = 0
            self._confs = []
            self._say("something there — stopping to look")
            return
        self._say("scanning")

    def _st_paused(self, detections, now):
        self._pause_ticks += 1
        if self._pause_ticks <= AUTO_PAUSE_SETTLE_TICKS:
            self._say("settling")
            return
        self._confs.append(max((d.conf for d in detections), default=0.0))
        avg = sum(self._confs) / len(self._confs)
        self._say(f"checking crack — conf {avg:.2f} "
                  f"({len(self._confs)}/{AUTO_PAUSE_CONFIRM_TICKS})")
        if len(self._confs) < AUTO_PAUSE_CONFIRM_TICKS:
            return
        if avg >= self.deploy_conf:
            self.state = "REPOSITIONING"
            self._reposition_start_mm = self.get_pos_mm()
            self._drive(1)
        else:
            # False alarm - a shadow or a pipe seam. Carry on.
            self.state = "SCANNING"
            self._drive(1)
            self._say("false alarm — scanning")

    def _st_repositioning(self, detections, now):
        pos = self.get_pos_mm()
        if self._reposition_start_mm is None or pos is None:
            # No way to measure the offset. Stop and deploy where we are
            # rather than drive forward blind.
            self._drive(0)
            self.state = "PRE_DEPLOY_WAIT"
            self._wait_started_t = now
            self._say("no telemetry — stopped early")
            return
        travelled = abs(pos - self._reposition_start_mm)
        self._say(f"repositioning {travelled:.0f}/{AUTO_REPOSITION_MM:.0f} mm")
        if travelled >= AUTO_REPOSITION_MM:
            self._drive(0)
            self.state = "PRE_DEPLOY_WAIT"
            self._wait_started_t = now

    def _st_pre_deploy_wait(self, detections, now):
        left = max(0.0, PRE_DEPLOY_WAIT_SECS - (now - self._wait_started_t))
        self._say(f"crack confirmed — deploying in {left:.1f}s")
        if left <= 0:
            self.state = "DEPLOYING"
            self.send("D")
            self._act_started_t = now

    def _st_deploying(self, detections, now):
        left = max(0.0, ACTUATOR_RUN_SECS - (now - self._act_started_t))
        self._say(f"deploying ({left:.1f}s left)")
        if left <= 0:
            self.send("X")
            self.state = "POST_DEPLOY_WAIT"
            self._wait_started_t = now

    def _st_post_deploy_wait(self, detections, now):
        left = max(0.0, POST_DEPLOY_WAIT_SECS - (now - self._wait_started_t))
        self._say(f"deployed — retracting in {left:.1f}s")
        if left <= 0:
            self.state = "RETRACTING_AUTO"
            self.send("R")
            self._act_started_t = now

    def _st_retracting_auto(self, detections, now):
        left = max(0.0, ACTUATOR_RUN_SECS - (now - self._act_started_t))
        self._say(f"retracting ({left:.1f}s left)")
        if left <= 0:
            self.send("X")
            self.state = "IDLE"
            self._say("sequence complete")

    def _st_returning_home(self, detections, now):
        # `detections` deliberately unused: homing must not stop for a crack.
        pos = self.get_pos_mm()
        if pos is None:
            self._drive(0)
            self.state = "IDLE"
            self._zero_on_arrival = False
            self._say("lost telemetry — stopped")
            return
        if abs(pos) <= HOME_TOLERANCE_MM:
            self._drive(0)
            self.state = "IDLE"
            if self._zero_on_arrival:
                self.send("Z")           # clear residual drift now it is home
                self._zero_on_arrival = False
                self._say("home — odometer zeroed")
            else:
                self._say("home")
            return
        # Re-checked every tick and direction-aware, so it self-corrects on
        # overshoot and works from either side of home.
        self._drive(-1 if pos > 0 else 1)
        self._say(f"returning home ({abs(pos):.0f} mm left)")

    # -- screenshots -------------------------------------------------------
    def maybe_capture(self, vis_bgr, det, tag):
        """One screenshot per crack ID, the first time its confidence lands in
        the review band while a scan is actually running.

        vis_bgr must already have this detection drawn on it.
        """
        if self.state not in SCREENSHOT_ACTIVE_STATES:
            return None
        if not (SCREENSHOT_CONF_MIN <= det.conf <= SCREENSHOT_CONF_MAX):
            return None
        key = (tag, det.track_id)
        if key in self._shot_ids:
            return None
        if not HAVE_CV2 or vis_bgr is None:
            return None
        self._shot_ids.add(key)

        pos = self.get_pos_mm()
        pos_txt = f"{pos:.0f}mm" if pos is not None else "unknown"
        img = vis_bgr.copy()
        h, w = img.shape[:2]
        caption = (f"{tag}{det.track_id} {det.label} conf {det.conf:.2f}"
                   f"  travelled {pos_txt}")
        cv2.rectangle(img, (0, h - 20), (w, h), (0, 0, 0), -1)
        cv2.putText(img, caption, (4, h - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (0, 255, 255), 1, cv2.LINE_AA)

        fname = f"crack_{tag}{det.track_id}_conf{det.conf:.2f}_pos{pos_txt}.png"
        path = os.path.join(self.ensure_dir(), fname)
        try:
            cv2.imwrite(path, img)
        except Exception as exc:                       # noqa: BLE001
            print(f"[auto] could not save {path}: {exc}")
            return None
        self.shots.append(path)
        print(f"flagged for review: {tag}{det.track_id} conf {det.conf:.2f} "
              f"@ {pos_txt} -> {path}")
        return path