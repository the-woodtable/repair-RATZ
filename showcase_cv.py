"""
showcase_cv.py — crack detection for rat88_panel.py

Everything CV-related lives here so the panel file stays about UI and serial.
Nothing in here touches the GUI or the serial port.

Detection runs on BOTH cameras independently (redundancy, not stereo depth --
see note below), matching control_panel_stereo2.py's cv_tracker_l/cv_tracker_r
pair. Camera-distance (stereo SGBM depth) was removed from this module for the
same reason it was removed from control_panel_stereo2.py: it was not accurate
enough to rely on. The only distance either panel reports now is the
stepper's own odometry (DISTANCE TRAVELED / the odometer readout), read via
telemetry, not computed from the cameras.

Every part degrades on its own:
    no CV.pt          -> video still shows, no boxes
    no ultralytics    -> video still shows, no boxes
    one camera only   -> that camera still shows, no detections from the
                         missing one
"""

import os
import threading
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WEIGHTS_FILE = os.path.join(HERE, "CV.pt")

# MUST match ROTATE_DEG in control_panel_stereo2.py. Kept for consistent
# on-screen orientation between the two panels -- not for stereo purposes,
# since there is no stereo pipeline here any more.
ROTATE_DEG = (90, 270)   # (left, right) — swapped with the L/R id swap

# Horizontal mirror per camera, 0 or 1. Applied AFTER rotation, so it means
# "flip what you are looking at left-to-right". Must match control_panel_
# stereo2.py so the two panels look the same to an operator switching
# between them.
MIRROR_H = (1, 1)


_ROTATE_FLAG = {90: cv2.ROTATE_90_CLOCKWISE,
                180: cv2.ROTATE_180,
                270: cv2.ROTATE_90_COUNTERCLOCKWISE}


def orient(img, cam):
    """Apply the per-camera rotation. cam is 'L' or 'R'."""
    i = 0 if cam == "L" else 1
    deg = ROTATE_DEG[i] % 360
    flag = _ROTATE_FLAG.get(deg)
    if flag is not None:
        img = cv2.rotate(img, flag)
    if MIRROR_H[i]:
        img = cv2.flip(img, 1)
    return img


# How much damage to tolerate before throwing a frame away. Same values the
# other panel uses, and they were tuned against real corrupt captures.
MAX_FLAT_BOTTOM = 0.05     # fraction of bottom rows allowed to be flat grey
MAX_COLOUR_JUMP = 45.0     # biggest allowed chroma step between two rows

# Rejection counters, so the panel can show how bad the link really is.
drops = {"L": 0, "R": 0}


def _flat_bottom_frac(img):
    """Fraction of BOTTOM rows that are a single flat value.

    A truncated capture decodes to real image on top and uniform grey below.
    Byte-level checks cannot catch these — the JPEG often still ends with a
    valid FFD9 — so this is the only reliable filter.
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    row_std = gray.std(axis=1)
    flat = 0
    for v in row_std[::-1]:
        if v < 2.0:
            flat += 1
        else:
            break
    return flat / gray.shape[0]


def _colour_band_jump(img):
    """Largest sudden CHROMA shift between consecutive rows.

    A JPEG bitstream error mid-frame corrupts the DC predictor, so every block
    after it inherits a wrong colour offset: normal on top, tinted below, with
    a hard horizontal edge. The bottom is not flat, so the truncation filter
    misses it entirely.

    Compares chroma rather than brightness on purpose. A real scene edge — a
    shadow line, a pipe rim — changes brightness while keeping its colour
    balance, and must not be rejected. A DC error shifts the channels by
    different amounts, which shows up in B-G / R-G even when brightness barely
    moves.
    """
    rows = img.reshape(img.shape[0], -1, 3).mean(axis=1)
    if len(rows) < 8:
        return 0.0
    chroma = np.stack([rows[:, 0] - rows[:, 1],      # B - G
                       rows[:, 2] - rows[:, 1]], 1)  # R - G
    diffs = np.abs(np.diff(chroma[3:-3], axis=0)).sum(axis=1)
    return float(diffs.max()) if len(diffs) else 0.0


def decode(jpeg_bytes, cam):
    """JPEG bytes -> oriented BGR ndarray, or None if the frame is corrupt.

    Returning None makes the panel hold its last good frame, which looks far
    better than flashing a torn one. This panel had NO corruption filter at
    all, so every damaged frame went straight to the screen — that is what
    "glitching" looked like.
    """
    arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        drops[cam] = drops.get(cam, 0) + 1
        return None

    # CHECK BEFORE ROTATING. Both detectors assume SENSOR orientation: JPEG
    # damage always affects the tail of the image in scan order, i.e. the
    # bottom rows. After a 90/270 rotation that tail becomes a vertical band
    # on the left or right edge, and a bottom-rows test would look straight
    # past it.
    if _flat_bottom_frac(img) > MAX_FLAT_BOTTOM:
        drops[cam] = drops.get(cam, 0) + 1
        return None
    if _colour_band_jump(img) > MAX_COLOUR_JUMP:
        drops[cam] = drops.get(cam, 0) + 1
        return None

    return orient(img, cam)


# --------------------------------------------------------------------------
# Worker
# --------------------------------------------------------------------------

class Shown:
    """One detection, ready to draw and to hand to the auto sequence.

    Named fields rather than a tuple because auto_sequence.py needs .conf,
    .track_id and .label by name, and track_id is what stops the same crack
    being screenshotted on every frame.
    """

    __slots__ = ("mask_xy", "label", "conf", "track_id")

    def __init__(self, mask_xy, label, conf, track_id):
        self.mask_xy = mask_xy
        self.label = label
        self.conf = conf
        self.track_id = track_id


class CVWorker(threading.Thread):
    """Runs YOLO on BOTH camera frames independently, off the GUI thread --
    redundancy (either camera can catch a crack the other misses), not
    stereo depth. Two separate CrackTracker instances, same as
    control_panel_stereo2.py's cv_tracker_l/cv_tracker_r: each camera's track
    IDs are their own independent identity space, so sharing one instance
    would mix up left/right detections under the same ID numbers.

    The panel never waits on this thread: it reads the newest result and
    draws whatever is there, so video stays smooth even when inference is
    slow.
    """

    def __init__(self, weights=WEIGHTS_FILE, conf=0.4):
        super().__init__(daemon=True)
        self.running = True
        self.conf = conf

        self.tracker_l = None
        self.tracker_r = None
        self.cv_error = None
        if not os.path.exists(weights):
            self.cv_error = f"no {os.path.basename(weights)} — detection off"
        else:
            try:
                from crack_tracker import CrackTracker
                self.tracker_l = CrackTracker(weights, conf_threshold=conf)
                self.tracker_r = CrackTracker(weights, conf_threshold=conf)
            except Exception as exc:
                self.cv_error = f"detection off: {exc}"

        self._in_lock = threading.Lock()
        self._pending_l = None
        self._pending_r = None

        self._out_lock = threading.Lock()
        self.detections_l = []
        self.detections_r = []
        self.infer_ms = 0.0

    def set_conf(self, value: float):
        """Change the detection threshold while running.

        Sets it on the tracker too, not just here: self.conf is only what the
        tracker was built with, and YOLO reads conf_threshold on every call.
        """
        self.conf = float(value)
        for tracker in (self.tracker_l, self.tracker_r):
            if tracker is not None:
                tracker.conf_threshold = float(value)

    def submit(self, left_bgr, right_bgr):
        """Called from the GUI thread. Keeps only the newest frame per
        camera; stale frames are dropped rather than queued, so the worker
        never falls behind the video."""
        with self._in_lock:
            if left_bgr is not None:
                self._pending_l = left_bgr
            if right_bgr is not None:
                self._pending_r = right_bgr

    def _take(self):
        with self._in_lock:
            l, r = self._pending_l, self._pending_r
            self._pending_l = self._pending_r = None
            return l, r

    def run(self):
        while self.running:
            l, r = self._take()
            if l is None and r is None:
                time.sleep(0.02)
                continue
            t0 = time.time()
            out_l, out_r = None, None
            if self.tracker_l is not None and l is not None:
                try:
                    dets = self.tracker_l.update(l)
                    out_l = [Shown(d.mask_xy, d.label, d.conf, d.track_id)
                            for d in dets]
                except Exception as exc:
                    self.cv_error = f"detection error (L): {exc}"
            if self.tracker_r is not None and r is not None:
                try:
                    dets = self.tracker_r.update(r)
                    out_r = [Shown(d.mask_xy, d.label, d.conf, d.track_id)
                            for d in dets]
                except Exception as exc:
                    self.cv_error = f"detection error (R): {exc}"
            with self._out_lock:
                if out_l is not None:
                    self.detections_l = out_l
                if out_r is not None:
                    self.detections_r = out_r
                self.infer_ms = (time.time() - t0) * 1000
            if out_l is None and out_r is None:
                time.sleep(0.01)

    def latest(self):
        """Returns (detections_l, detections_r, infer_ms)."""
        with self._out_lock:
            return list(self.detections_l), list(self.detections_r), self.infer_ms

    def stop(self):
        self.running = False


# --------------------------------------------------------------------------
# Drawing
# --------------------------------------------------------------------------

MASK_COLOR = (118, 230, 0)      # BGR, matches the panel's green
TEXT_COLOR = (255, 255, 255)


def draw_conf_badge(frame_bgr, det_conf, deploy_conf):
    """Print BOTH thresholds in the corner of the frame.

    Both, because they are easy to confuse and they answer different
    questions. det is why a box is or is not drawn; deploy is why the robot
    did or did not commit a lining to a crack you can plainly see.

    On the video rather than in the status strip because the strip is hidden
    during the showcase, and a threshold you cannot see is one you cannot
    trust: "no cracks here" and "the bar is too high" look identical
    otherwise.

    Both are fixed at runtime; this is a readout, not a control.
    """
    out = frame_bgr
    h = out.shape[0]
    txt = f"det {det_conf:.2f}  deploy {deploy_conf:.2f}"
    (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
    x, y = 6, h - 8
    bg, fg = (0, 0, 0), (235, 235, 235)
    cv2.rectangle(out, (x - 4, y - th - 6), (x + tw + 4, y + 4), bg, -1)
    cv2.putText(out, txt, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, fg, 1,
                cv2.LINE_AA)
    return out


def draw_detections(frame_bgr, detections):
    """Draw masks and labels onto a copy of the frame. No distance here --
    camera-distance was removed as unreliable; DISTANCE TRAVELED (stepper
    odometry) is the only distance either panel reports."""
    if not detections:
        return frame_bgr
    out = frame_bgr.copy()
    overlay = out.copy()
    for d in detections:
        pts = np.asarray(d.mask_xy, dtype=np.int32).reshape(-1, 1, 2)
        cv2.fillPoly(overlay, [pts], MASK_COLOR)
        cv2.polylines(out, [pts], True, MASK_COLOR, 2)
    cv2.addWeighted(overlay, 0.35, out, 0.65, 0, out)

    for d in detections:
        pts = np.asarray(d.mask_xy, dtype=np.int32)
        x, y = int(pts[:, 0].min()), int(pts[:, 1].min())
        txt = f"{d.label} {d.conf:.2f}"
        (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        y = max(y, th + 6)
        cv2.rectangle(out, (x, y - th - 6), (x + tw + 6, y), MASK_COLOR, -1)
        cv2.putText(out, txt, (x + 3, y - 4), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (20, 20, 20), 1, cv2.LINE_AA)
    return out