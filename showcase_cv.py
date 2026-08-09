"""
showcase_cv.py — crack detection + stereo depth for rat88_panel.py

Everything CV-related lives here so the panel file stays about UI and serial.
Nothing in here touches the GUI or the serial port.

The stereo maths is copied from control_panel_stereo2.py deliberately rather
than imported, because that module builds a Tk app at import time. If you
retune SGBM or change ROTATE_DEG in one file, change it in the other too.

Every part degrades on its own:
    no CV.pt          -> video still shows, no boxes
    no ultralytics    -> video still shows, no boxes
    no stereo_calib   -> boxes still show, distance reads "--"
    one camera only   -> that camera still shows, no distance
"""

import os
import threading
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WEIGHTS_FILE = os.path.join(HERE, "CV.pt")
CALIB_FILE = os.path.join(HERE, "stereo_calib.npz")

# MUST match ROTATE_DEG in control_panel_stereo2.py and stereo_calibrate.py.
# Not cosmetic: the disparity search only looks sideways, so both images must
# be upright with the cameras separated along the image's horizontal axis.
ROTATE_DEG = (270, 90)   # (left, right)

_ROTATE_FLAG = {90: cv2.ROTATE_90_CLOCKWISE,
                180: cv2.ROTATE_180,
                270: cv2.ROTATE_90_COUNTERCLOCKWISE}


def orient(img, cam):
    """Apply the per-camera rotation. cam is 'L' or 'R'."""
    deg = ROTATE_DEG[0 if cam == "L" else 1] % 360
    flag = _ROTATE_FLAG.get(deg)
    return img if flag is None else cv2.rotate(img, flag)


def decode(jpeg_bytes, cam):
    """JPEG bytes -> oriented BGR ndarray, or None if the frame is corrupt."""
    arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    return None if img is None else orient(img, cam)


# --------------------------------------------------------------------------
# Stereo
# --------------------------------------------------------------------------

class StereoCalib:
    """Loads stereo_calib.npz and turns a rectified pair into distances.

    numDisparities sets how far SGBM searches sideways, and therefore the
    CLOSEST measurable distance:  nearest_mm = fx * baseline / numDisparities.
    It also blanks the leftmost numDisparities columns, so it must stay below
    ~40% of frame width or the centre of the image loses its readout. 96 is
    the value tuned in control_panel_stereo2.py; keep them in step.
    """

    def __init__(self, path=CALIB_FILE):
        self.ok = False
        self.error = None
        if not os.path.exists(path):
            self.error = "no stereo_calib.npz — run stereo_calibrate.py"
            return
        try:
            d = np.load(path)
            self.map1x, self.map1y = d["map1x"], d["map1y"]
            self.map2x, self.map2y = d["map2x"], d["map2y"]
            self.fx = float(d["fx"])
            self.baseline = float(d["baseline"])          # mm
            self.sgbm = cv2.StereoSGBM_create(
                minDisparity=0, numDisparities=96, blockSize=7,
                P1=8 * 49, P2=32 * 49, uniquenessRatio=10,
                speckleWindowSize=100, speckleRange=2, disp12MaxDiff=1)
            self.ok = True
        except KeyError:
            self.error = ("stereo_calib.npz is an old format — "
                          "re-run stereo_calibrate.py")
        except Exception as exc:
            self.error = f"stereo_calib.npz unreadable: {exc}"

    def rectify(self, left, right):
        if not self.ok:
            return left, right
        return (self.rectify_one(left, "L"), self.rectify_one(right, "R"))

    def rectify_one(self, img, cam):
        """Rectify a single frame. The maps are per-camera, so frames can be
        rectified as they arrive instead of waiting to have a matched pair.

        Everything downstream — display, detection, disparity — then lives in
        rectified coordinates, so a detection centroid can index the disparity
        map directly. Mixing the two spaces silently returns wrong distances.
        """
        if not self.ok or img is None:
            return img
        if cam == "L":
            return cv2.remap(img, self.map1x, self.map1y, cv2.INTER_LINEAR)
        return cv2.remap(img, self.map2x, self.map2y, cv2.INTER_LINEAR)

    def disparity(self, rect_l, rect_r):
        gl = cv2.cvtColor(rect_l, cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(rect_r, cv2.COLOR_BGR2GRAY)
        return self.sgbm.compute(gl, gr).astype(np.float32) / 16.0

    def distance_mm(self, disp, x, y):
        """Median disparity over a 9x9 patch -> mm, or None if too few pixels."""
        if disp is None:
            return None
        h, w = disp.shape
        x, y = int(x), int(y)
        if not (0 <= x < w and 0 <= y < h):
            return None
        x0, x1 = max(0, x - 4), min(w, x + 5)
        y0, y1 = max(0, y - 4), min(h, y + 5)
        patch = disp[y0:y1, x0:x1]
        good = patch[patch > 0.5]
        if good.size < 10:
            return None
        return self.fx * self.baseline / float(np.median(good))


# --------------------------------------------------------------------------
# Worker
# --------------------------------------------------------------------------

class CVWorker(threading.Thread):
    """Runs YOLO on the LEFT frame and SGBM on the pair, off the GUI thread.

    SGBM is the expensive half, so it runs at its own slower cadence. The
    panel never waits on this thread: it reads the newest result and draws
    whatever is there, so video stays smooth even when inference is slow.
    """

    def __init__(self, weights=WEIGHTS_FILE, calib_path=CALIB_FILE,
                 conf=0.4, depth_period=0.2):
        super().__init__(daemon=True)
        self.running = True
        self.conf = conf
        self.depth_period = depth_period

        self.calib = StereoCalib(calib_path)
        self.tracker = None
        self.cv_error = None
        if not os.path.exists(weights):
            self.cv_error = f"no {os.path.basename(weights)} — detection off"
        else:
            try:
                from crack_tracker import CrackTracker
                self.tracker = CrackTracker(weights, conf_threshold=conf)
            except Exception as exc:
                self.cv_error = f"detection off: {exc}"

        self._in_lock = threading.Lock()
        self._pending_l = None
        self._pending_r = None

        self._out_lock = threading.Lock()
        self.detections = []       # list of (mask_xy, label, conf, dist_mm)
        self.disp = None
        self.infer_ms = 0.0
        self.depth_ms = 0.0

    def submit(self, left_bgr, right_bgr):
        """Called from the GUI thread with ALREADY-RECTIFIED frames.
        Keeps only the newest pair; stale frames are dropped rather than
        queued, so the worker never falls behind the video."""
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
        last_depth = 0.0
        last_l = last_r = None
        while self.running:
            l, r = self._take()
            if l is None and r is None:
                time.sleep(0.02)
                continue
            if l is not None:
                last_l = l
            if r is not None:
                last_r = r

            # --- depth, on its own slower clock ---------------------------
            now = time.time()
            if (self.calib.ok and last_l is not None and last_r is not None
                    and now - last_depth >= self.depth_period):
                last_depth = now
                t0 = time.time()
                try:
                    # Frames arrive rectified, so SGBM can run directly.
                    d = self.calib.disparity(last_l, last_r)
                    with self._out_lock:
                        self.disp = d
                        self.depth_ms = (time.time() - t0) * 1000
                except Exception:
                    with self._out_lock:
                        self.disp = None

            # --- detection on the left frame only -------------------------
            if self.tracker is not None and last_l is not None:
                t0 = time.time()
                try:
                    dets = self.tracker.update(last_l)
                except Exception as exc:
                    self.cv_error = f"detection error: {exc}"
                    dets = []
                with self._out_lock:
                    disp = self.disp
                out = []
                for det in dets:
                    cx, cy = det.centroid
                    dist = self.calib.distance_mm(disp, cx, cy) if disp is not None else None
                    out.append((det.mask_xy, det.label, det.conf, dist))
                with self._out_lock:
                    self.detections = out
                    self.infer_ms = (time.time() - t0) * 1000
            else:
                time.sleep(0.01)

    def latest(self):
        with self._out_lock:
            return list(self.detections), self.infer_ms, self.depth_ms

    def stop(self):
        self.running = False


# --------------------------------------------------------------------------
# Drawing
# --------------------------------------------------------------------------

MASK_COLOR = (118, 230, 0)      # BGR, matches the panel's green
TEXT_COLOR = (255, 255, 255)


def draw_detections(frame_bgr, detections):
    """Draw masks, labels and distances onto a copy of the frame."""
    if not detections:
        return frame_bgr
    out = frame_bgr.copy()
    overlay = out.copy()
    for mask_xy, label, conf, dist in detections:
        pts = np.asarray(mask_xy, dtype=np.int32).reshape(-1, 1, 2)
        cv2.fillPoly(overlay, [pts], MASK_COLOR)
        cv2.polylines(out, [pts], True, MASK_COLOR, 2)
    cv2.addWeighted(overlay, 0.35, out, 0.65, 0, out)

    for mask_xy, label, conf, dist in detections:
        pts = np.asarray(mask_xy, dtype=np.int32)
        x, y = int(pts[:, 0].min()), int(pts[:, 1].min())
        txt = f"{label} {conf:.2f}"
        if dist is not None:
            txt += f"  {dist / 10.0:.1f} cm"
        (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        y = max(y, th + 6)
        cv2.rectangle(out, (x, y - th - 6), (x + tw + 6, y), MASK_COLOR, -1)
        cv2.putText(out, txt, (x + 3, y - 4), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (20, 20, 20), 1, cv2.LINE_AA)
    return out
