"""
control_panel_stereo2.py — EDI PIPE CAM camera panel
=====================================================
Capture and calibration only. No motor / servo / actuator controls —
robot driving lives in the teammate's panel.

What this does:
  * Live left/right video from the two ESP32-CAMs, each running its own
    crack/hole detector (redundancy, not stereo depth -- see note below)
  * REC          -> record both feeds to MP4 (footage review)
  * SAVE / AUTO-CAP -> synchronized PNG stills (YOLO training set)
  * CALIB PAIR   -> checkerboard pairs, kept for a future stereo rebuild

Distance is reported ONLY from the stepper's own odometry (telemetry's
pos_mm / steps) via DISTANCE TRAVELED. Stereo camera-distance and IMU
telemetry (imu_pos/imu_vel/imu_bias/slip) were removed from this panel --
they were not accurate enough to rely on.

Works with either S3 firmware (cameras-only sketch or the full PlatformIO
build) — both send the same frame protocol:
    0xAA 0x55 | id(1) | len(4 LE) | payload
    id 0 = LEFT JPEG, 1 = RIGHT JPEG, 2 = telemetry JSON (ignored here)

Run:
    source ~/Downloads/esp32env/bin/activate
    cd ~/Desktop/30.007/"camera codes"
    python3 control_panel_stereo2.py

Deps: pip install pyserial opencv-python pillow numpy
"""

import json
import os
import queue
import re
import threading
import time
import tkinter as tk

import cv2
import numpy as np
import serial
import serial.tools.list_ports
from PIL import Image, ImageTk

from crack_tracker import CrackTracker

# ============================ CONFIG ============================
HERE = os.path.dirname(os.path.abspath(__file__))

PORT   = None          # None = auto-detect (cross-platform, by VID), or force e.g. "COM7"
BAUD   = 921600        # ignored by USB CDC but pyserial requires it
SCALE  = 2             # display zoom (320x240 -> 640x480)

# All captured data lives under one folder in the course directory so it
# is easy to find, back up, and hand in.
DATA_DIR    = os.path.expanduser("~/Desktop/30.007/pipe_cam_data")
REC_DIR     = os.path.join(DATA_DIR, "recordings")
DATASET_DIR = os.path.join(DATA_DIR, "dataset")
CALIB_DIR   = os.path.join(DATA_DIR, "calib_pairs")

REC_FPS      = 10
AUTOCAP_SECS = 1.0
PAIR_MAX_AGE = 0.30    # L/R further apart than this = unsynced
# Reject a decoded frame if more than this fraction of its bottom rows are
# flat grey — that's a truncated capture. 0.05 = 5%. Raise toward 0.5 if
# you'd rather see partial frames than have the display freeze.
MAX_FLAT_BOTTOM = 0.05
# Reject a frame if the mean colour jumps by more than this between two
# neighbouring rows — that hard horizontal edge means a JPEG bitstream error
# corrupted everything below it (shows up as a sudden purple/green band).
# Raise if legitimate scenes are being rejected; lower to be stricter.
MAX_COLOUR_JUMP = 45.0

# Crack/hole detection model. Set to your trained weights path.
CV_MODEL_PATH = os.path.join(HERE, "CV.pt")
CV_CONF = 0.2

# ---- autonomous scan loop ----
# This is a SEPARATE, higher bar than CV_CONF above. CV_CONF (and the "conf"
# slider) controls what the tracker bothers to report at all. This constant
# controls the much bigger decision of "stop and reposition for a human to
# look at this" -- a false positive here wastes travel distance and an
# operator's attention, so it's set high on purpose.
AUTO_DEPLOY_CONF = 0.8
# Lower bar: a detection needs at least this much confidence to make the
# robot stop and take a closer look at all. Below this, it's ignored and
# scanning continues -- only detections at or above this get a second look.
AUTO_PAUSE_TRIGGER_CONF = 0.5
# How far to drive FORWARD, past a confirmed crack, before deploying, in mm.
AUTO_REPOSITION_MM = 30.0
# Wait after reaching the reposition point, before extending the actuator.
PRE_DEPLOY_WAIT_SECS = 5.0
# Wait after the lining is fully deployed, before retracting.
POST_DEPLOY_WAIT_SECS = 5.0
# "Deploy/retract finished" is detected generically, without needing to know
# what main.cpp's actuator.getState() numbers actually mean: once telemetry's
# "actuator" field stops CHANGING for this many seconds, the actuator is
# assumed to have stopped moving (fully extended/retracted, or stalled at
# the limit). Used for BOTH the deploy and the retract half of the sequence.
ACTUATOR_STABLE_SECS = 0.5
# Safety cap regardless of the above, in case the actuator field never
# settles for some reason -- don't wait forever.
ACTUATOR_TIMEOUT_SECS = 8.0
# If telemetry has no "actuator" field at all, there's nothing to watch --
# just wait this long and assume the move finished.
ACTUATOR_NO_TELEM_WAIT_SECS = 3.0
# How close to "home" (position 0, i.e. wherever ZERO ODOM was last pressed)
# counts as arrived, when driving back via the RETURN HOME button, in mm.
HOME_TOLERANCE_MM = 2.0
# After stopping to look at a detection, wait this many ticks (40ms each)
# before trusting any confidence reading -- the frame is still motion-blurred
# from driving for the first few ticks after 'S' is sent.
AUTO_PAUSE_SETTLE_TICKS = 5
# Then average confidence over this many ticks before deciding. Averaging
# beats trusting a single frame, since a single frame can spike or drop from
# lighting flicker or a partial view of the crack.
AUTO_PAUSE_CONFIRM_TICKS = 8
# Fallback conversion if telemetry ever has 'steps' but no 'pos_mm' -- from
# the calibration run: 2352 steps measured over 730mm of travel.
STEPS_PER_MM_FALLBACK = 2352 / 730.0

# Rotation applied to each camera on arrival: 0, 90, 180 or 270 clockwise.
# PER CAMERA, because the two are not necessarily mounted the same way —
# ROTATE_DEG = (left, right).
#
# Applied before display, recording, stills AND calibration, so everything
# downstream sees one consistent orientation.
#
# THIS IS NOT COSMETIC. Stereo depth needs both images upright and with the
# cameras separated along the image's HORIZONTAL axis — the disparity search
# only looks sideways. Mismatched or 90-degree-rotated frames make depth
# impossible to compute.
#
# Ideally fix the MOUNTS so both cameras sit the same way and set this to
# (0, 0); software rotation costs a little CPU per frame and is one more
# thing to keep in sync with stereo_calibrate.py.
# (270, 90) + 180 on each = (90, 270). Both views were upside down.
# The 180-degree offset BETWEEN the two cameras is unchanged, so they still
# agree with each other — only their shared orientation flipped.
# MUST match stereo_calibrate.py exactly or every distance is silently wrong.
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


# ============================ SERIAL ============================
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


class FrameLink:
    """Reader thread parses frames; a second thread decodes JPEGs.

    Decoding must NOT happen in the reader — it takes long enough that the
    USB buffer overflows meanwhile, which shows up as torn frames.
    """

    MAGIC = b"\xAA\x55"
    ID_MAP = {b"\x00": 0, b"\x01": 1, b"L": 0, b"R": 1}
    TELEM_ID = b"\x02"
    MAX_FRAME = 300_000

    def __init__(self, ser):
        self.ser = ser
        self.latest = {0: None, 1: None}
        self.telem = {}
        self.counts = {0: 0, 1: 0}      # frames accepted, for the fps readout
        self.dropped = 0                # frames rejected as corrupt (total)
        # Live diagnostics — same measurements stream_quality.py makes, but
        # gathered continuously so you can debug WITHOUT closing the panel.
        # (Only one program can hold the serial port, so they must live here.)
        self.bytes_total = 0            # everything read off the wire
        self.bytes_frames = 0           # bytes that became a usable frame
        self.telem_count = 0
        self.drop_badstart = 0          # header said frame, payload wasn't JPEG
        self.drop_spliced = 0           # end marker far too early
        self.drop_decode = 0            # JPEG wouldn't decode at all
        self.drop_flat = 0              # decoded but truncated (grey bottom)
        self.drop_band = 0              # colour-band edge (bitstream error)
        self.last_stats = {}            # per-camera brightness/contrast
        self.port_error = None          # set if the port dies
        self._textbuf = bytearray()     # loose ASCII between frames
        # Colour-drift tracking. chan_ref is the very first frame from each
        # camera; comparing "now" against it turns "it looks purple" into a
        # number, and shows whether the cast is growing or was always there.
        self.chan_stats = {0: None, 1: None}   # (R, G, B) means, latest frame
        self.chan_ref = {0: None, 1: None}     # (R, G, B) means, first frame
        self.chan_t0 = {0: None, 1: None}      # timestamp of that first frame
        self.chan_log = None                   # set to a file handle to record
        self.lock = threading.Lock()
        self.running = True
        self._jpegq = queue.Queue(maxsize=4)
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._decoder, daemon=True).start()

    def send(self, s):
        try:
            self.ser.write(s.encode())
        except serial.SerialException:
            pass

    # Matches  "key":value  for numbers, regardless of what separates the
    # pairs. main.cpp currently emits `{"` between fields where it should
    # emit `,"`, so the line is not valid JSON and json.loads() rejects it
    # outright. Scraping pairs works on the broken AND the fixed output.
    _TELEM_RE = re.compile(rb'"(\w+)"\s*:\s*(-?\d+(?:\.\d+)?)')

    def _take_text_line(self):
        line = bytes(self._textbuf)
        self._textbuf.clear()
        pairs = self._TELEM_RE.findall(line)
        if not pairs:
            return
        d = {}
        for k, v in pairs:
            v = v.decode()
            d[k.decode()] = float(v) if "." in v else int(v)
        with self.lock:
            self.telem = d
            self.telem_count += 1

    def _read_exact(self, n):
        buf = bytearray()
        while self.running and len(buf) < n:
            chunk = self.ser.read(n - len(buf))
            if chunk:
                buf.extend(chunk)
        self.bytes_total += len(buf)
        return bytes(buf)

    def _reader(self):
        sync = bytearray()
        while self.running:
            try:
                b = self.ser.read(1)
            except serial.SerialException as e:
                # Port died (unplugged, S3 reset, brownout). Record it so the
                # panel can SAY so instead of just going quiet.
                self.port_error = str(e)
                self.running = False
                return
            if not b:
                continue
            self.bytes_total += 1
            sync += b
            if len(sync) > 2:
                # This byte can't be the start of a magic pair, so it's
                # loose text — main.cpp's sendTelemetry() prints raw, outside
                # the frame protocol, so this is where telemetry arrives.
                self._textbuf.append(sync.pop(0))
            if b in (b"\n", b"\r"):
                # A newline can't be part of the magic, so release whatever
                # is parked in the sync window before parsing the line.
                self._textbuf.extend(sync)
                sync.clear()
                self._take_text_line()
            elif len(self._textbuf) > 4096:
                del self._textbuf[:-512]   # runaway binary, don't grow forever
            if bytes(sync) != self.MAGIC:
                continue
            sync.clear()

            head = self._read_exact(5)
            if len(head) < 5:
                continue
            raw_id = head[:1]
            length = int.from_bytes(head[1:5], "little")
            if not (0 < length <= self.MAX_FRAME):
                continue

            # Telemetry must be consumed, not skipped, or its bytes get
            # mistaken for image data. (Not displayed in this panel.)
            if raw_id == self.TELEM_ID:
                body = self._read_exact(length)
                self.telem_count += 1
                try:
                    with self.lock:
                        self.telem = json.loads(body.decode("ascii", "ignore"))
                except json.JSONDecodeError:
                    pass
                continue

            cam = self.ID_MAP.get(raw_id)
            if cam is None:
                continue

            # Peek the first 2 bytes before committing to `length`. If the
            # header was corrupted, bailing here costs 2 bytes; reading the
            # full bogus length would swallow the next real frames whole
            # (that's what made one glitch freeze the stream for seconds).
            head2 = self._read_exact(2)
            if head2 != b"\xFF\xD8":
                self.dropped += 1
                self.drop_badstart += 1
                continue

            payload = head2 + self._read_exact(length - 2)
            if len(payload) != length:
                continue

            # Sensor pads bytes after FFD9; an end marker far too early means
            # the frame is spliced from pieces of different frames.
            end = payload.rfind(b"\xFF\xD9")
            if end < 0 or end + 2 < length * 0.5:
                self.dropped += 1
                self.drop_spliced += 1
                continue

            self.bytes_frames += length + 7
            try:
                self._jpegq.put_nowait((cam, payload[:end + 2], time.time()))
            except queue.Full:
                pass

    @staticmethod
    def _colour_band_jump(img):
        """Largest sudden colour shift between consecutive image rows.

        A JPEG bitstream error mid-frame corrupts the DC predictor, so every
        block AFTER the error inherits a wrong colour offset — the image goes
        normal-on-top, tinted-below with a hard horizontal edge. The bottom
        isn't flat, so the grey-bottom filter misses it; this catches it by
        looking for an abrupt jump in mean colour from one row to the next.
        """
        rows = img.reshape(img.shape[0], -1, 3).mean(axis=1)   # per-row B,G,R
        if len(rows) < 8:
            return 0.0
        # Compare CHROMA (colour balance), not total brightness. A real scene
        # edge — a horizon, a shadow line — changes brightness while keeping
        # its colour balance, so it must NOT trigger this. A DC-predictor
        # error shifts the channels by different amounts, which shows up as a
        # jump in B-G / R-G even when overall brightness barely moves.
        chroma = np.stack([rows[:, 0] - rows[:, 1],      # B - G
                           rows[:, 2] - rows[:, 1]], 1)  # R - G
        diffs = np.abs(np.diff(chroma[3:-3], axis=0)).sum(axis=1)
        return float(diffs.max()) if len(diffs) else 0.0

    @staticmethod
    def _flat_bottom_frac(img):
        """Fraction of rows at the BOTTOM that are a single flat value.

        A truncated capture decodes to real image on top and uniform grey
        below. Byte-level checks can't catch these (the JPEG may still end
        with a valid FFD9), so this is the only reliable filter.
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

    def _decoder(self):
        while self.running:
            try:
                cam, jpg, ts = self._jpegq.get(timeout=0.2)
            except queue.Empty:
                continue
            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                self.dropped += 1
                self.drop_decode += 1
                continue
            # Correct a rotated MOUNT here, once, before anything else sees
            # the frame — so display, recording, stills and calibration all
            # share one orientation.
            # ---- REJECT CORRUPT FRAMES *BEFORE* ROTATING ----
            # Both detectors assume SENSOR orientation: JPEG damage always
            # affects the tail of the image in scan order, i.e. the bottom
            # rows. After a 90/270 rotation that tail becomes a vertical band
            # on the left or right edge, so a bottom-rows test and a
            # row-to-row seam test both look at the wrong axis and pass the
            # corruption straight through. Checking first costs nothing and
            # keeps the detectors honest whatever ROTATE_DEG is set to.
            if self._flat_bottom_frac(img) > MAX_FLAT_BOTTOM:
                self.dropped += 1
                self.drop_flat += 1
                continue
            if self._colour_band_jump(img) > MAX_COLOUR_JUMP:
                self.dropped += 1
                self.drop_band += 1
                continue

            deg = ROTATE_DEG[cam] % 360
            if deg:
                img = cv2.rotate(img, _ROTATE_FLAG[deg])
            if MIRROR_H[cam]:
                img = cv2.flip(img, 1)
            # Cheap per-camera image stats so the panel can show whether the
            # two cameras are exposed the same (needed for stereo matching).
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Per-channel means. A colour cast is a question of WHICH channel
            # moves, and grayscale throws that away: magenta is green falling,
            # not red and blue rising, and the two have different causes.
            b, g, r = (float(img[:, :, i].mean()) for i in range(3))
            with self.lock:
                self.latest[cam] = (img, ts)
                self.counts[cam] += 1
                self.last_stats[cam] = (float(gray.mean()), float(gray.std()))
                self.chan_stats[cam] = (r, g, b)
                if self.chan_t0[cam] is None:      # first frame = reference
                    self.chan_ref[cam] = (r, g, b)
                    self.chan_t0[cam] = ts
                if self.chan_log is not None:
                    self.chan_log.write(
                        f"{ts:.3f},{cam},{r:.2f},{g:.2f},{b:.2f}\n")

    def get_pair(self):
        with self.lock:
            l, r = self.latest[0], self.latest[1]
        if l is None or r is None:
            return (l[0] if l else None), (r[0] if r else None), False
        return l[0], r[0], abs(l[1] - r[1]) <= PAIR_MAX_AGE

    def close(self):
        self.running = False


class CamWorker(threading.Thread):
    """Hands off the latest L/R frame pair to the GUI thread at a steady
    rate. No stereo rectification or disparity here anymore -- that
    distance-from-cameras readout was never reliable enough to trust, so
    it's gone. The only distance this panel reports now is the stepper's
    own odometry (DISTANCE TRAVELED, from telemetry)."""

    def __init__(self, link):
        super().__init__(daemon=True)
        self.link = link
        self.lock = threading.Lock()
        self.view_l = self.view_r = None
        self.running = True

    def run(self):
        while self.running:
            l, r, _ = self.link.get_pair()
            if l is None and r is None:
                time.sleep(0.05)
                continue
            with self.lock:
                self.view_l, self.view_r = l, r
            time.sleep(0.03)

    def snapshot(self):
        with self.lock:
            return self.view_l, self.view_r


# ============================ GUI ============================
class App:
    def __init__(self, root, link):
        self.root, self.link = root, link
        self.worker = CamWorker(link)
        self.worker.start()

        # Crack/hole detector — runs on the LEFT view each tick. Loaded
        # once here (model load is slow); update() itself is fast enough
        # to call every frame.
        self.cv_tracker_l = None
        self.cv_tracker_r = None
        self.cv_error = None
        if os.path.exists(CV_MODEL_PATH):
            try:
                # Separate tracker instances per camera -- each camera's
                # track IDs are their own independent identity space, so
                # sharing one instance would mix up left/right detections
                # under the same ID numbers.
                self.cv_tracker_l = CrackTracker(CV_MODEL_PATH, conf_threshold=CV_CONF)
                self.cv_tracker_r = CrackTracker(CV_MODEL_PATH, conf_threshold=CV_CONF)
            except Exception as e:
                self.cv_error = str(e)
        else:
            self.cv_error = f"model not found: {CV_MODEL_PATH}"

        root.title("EDI PIPE CAM — camera panel (capture + calibration)")
        root.configure(bg="#1e1e1e")

        # ---- scrollable body ----
        # ROTATE_DEG turns the 320x240 frames portrait, so at SCALE 2 the two
        # views alone are 640 px wide by 960 tall — taller than a laptop
        # screen once the controls are stacked underneath. Everything lives
        # in a scrolling canvas so the diagnostics are always reachable.
        outer = tk.Frame(root, bg="#1e1e1e")
        outer.pack(fill="both", expand=True)
        canvas = tk.Canvas(outer, bg="#1e1e1e", highlightthickness=0)
        vbar = tk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vbar.set)
        vbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        body = tk.Frame(canvas, bg="#1e1e1e")
        win = canvas.create_window((0, 0), window=body, anchor="nw")
        body.bind("<Configure>",
                  lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        # Keep the inner frame as wide as the canvas so fill="x" still works.
        canvas.bind("<Configure>",
                    lambda e: canvas.itemconfigure(win, width=e.width))

        def _wheel(e):
            # macOS gives small delta units; X11 sends Button-4/5 instead.
            canvas.yview_scroll(-1 * (e.delta if abs(e.delta) < 10
                                      else e.delta // 120), "units")
        canvas.bind_all("<MouseWheel>", _wheel)
        canvas.bind_all("<Button-4>", lambda e: canvas.yview_scroll(-3, "units"))
        canvas.bind_all("<Button-5>", lambda e: canvas.yview_scroll(3, "units"))

        # Open no taller than the screen, so there is always a scrollbar to
        # grab rather than a window whose bottom is off the display.
        root.update_idletasks()
        root.geometry(f"{min(1400, root.winfo_screenwidth() - 40)}x"
                      f"{root.winfo_screenheight() - 120}")

        # ---- video ----
        vid = tk.Frame(body, bg="#1e1e1e")
        vid.pack(padx=8, pady=8)
        self.lblL = tk.Label(vid, text="waiting for LEFT...", bg="black",
                             fg="#888", width=44, height=16)
        self.lblR = tk.Label(vid, text="waiting for RIGHT...", bg="black",
                             fg="#888", width=44, height=16)
        self.lblL.grid(row=0, column=0, padx=4)
        self.lblR.grid(row=0, column=1, padx=4)

        # How far the ROBOT has traveled, from telemetry's pos_mm (i.e.
        # steps / STEPS_PER_MM on the firmware) -- the ONLY distance this
        # panel reports. Made large/bold/cyan so it's easy to spot at a
        # glance while driving, unlike the small diagnostics text below.
        self.lbl_traveled = tk.Label(body, text="DISTANCE TRAVELED: -- cm",
                                     bg="#1e1e1e", fg="#0ff",
                                     font=("Menlo", 22, "bold"))
        self.lbl_traveled.pack(pady=(4, 8))

        self.lbl_status = tk.Label(body, text="", bg="#1e1e1e", fg="#0f0")
        self.lbl_status.pack()

        # ---- crack/hole detections ----
        cvf = tk.LabelFrame(body, text="Crack/Hole Detections",
                            bg="#1e1e1e", fg="#ccc")
        cvf.pack(fill="x", padx=8, pady=4)

        conf_row = tk.Frame(cvf, bg="#1e1e1e")
        conf_row.pack(fill="x", padx=6, pady=(4, 0))
        tk.Label(conf_row, text="conf", bg="#1e1e1e", fg="#ccc").pack(side="left")
        self.cv_conf_scale = tk.Scale(
            conf_row, from_=0.05, to=0.95, resolution=0.05, orient="horizontal",
            bg="#1e1e1e", fg="#ccc", highlightthickness=0, troughcolor="#333",
            length=220, command=self._on_conf_scale)
        self.cv_conf_scale.set(CV_CONF)
        self.cv_conf_scale.pack(side="left", padx=6)

        self.lbl_cv = tk.Label(cvf, text="", bg="#1e1e1e", fg="#f66",
                               font=("Menlo", 11), justify="left", anchor="w")
        self.lbl_cv.pack(fill="x", padx=6, pady=4)
        if self.cv_error:
            self.lbl_cv.config(text=f"CV disabled: {self.cv_error}", fg="#e8a33d")

        # ---- robot controls ----
        # These send the same single characters main.cpp listens for. They
        # live here because only ONE program can hold the serial port — you
        # can't type into a serial monitor while the panel is streaming.
        rob = tk.LabelFrame(body, text="Robot", bg="#1e1e1e", fg="#ccc")
        rob.pack(fill="x", padx=8, pady=4)

        # Drive: TOGGLE, not hold-to-move. Click once to start moving that
        # direction, click the same button again (or STOP ALL) to stop.
        # The firmware has no dead-man watchdog either way -- it just runs
        # until told 'S' -- so toggle mode isn't giving up any safety the
        # firmware already had; it just removes the "must hold the mouse
        # down" requirement the old hold-to-move buttons added on top.
        self.drive_state = 0   # 0 = stopped, +1 = forward, -1 = backward
        self.fwd_btn = tk.Button(rob, text="▲ FWD", width=10,
                                 command=self._toggle_forward)
        self.fwd_btn.grid(row=0, column=0, padx=3, pady=3)
        self.back_btn = tk.Button(rob, text="▼ BACK", width=10,
                                  command=self._toggle_backward)
        self.back_btn.grid(row=0, column=1, padx=3, pady=3)
        # NOTE: macOS Tk ignores `bg` on a Button and draws the native white
        # widget, so bg="#8b0000" + fg="white" rendered as white-on-white —
        # an invisible panic button. Colour the TEXT instead; that is honoured
        # on every platform.
        tk.Button(rob, text="■ STOP ALL", width=10, fg="#b00000",
                  highlightbackground="#8b0000",
                  command=self.stop_all).grid(row=0, column=2, padx=3, pady=3)

        # Actuator: hold-to-move for MANUAL use. guard_auto=True means these
        # ignore manual presses while the autonomous crack sequence owns the
        # actuator (deploy/retract there are fully automatic -- see below) --
        # otherwise a manual click mid-sequence could double-fire the motor.
        self.deploy_btn = self._hold_btn(rob, "DEPLOY", "D", "X", 0, 3,
                                         guard_auto=True)
        self.retract_btn = self._hold_btn(rob, "RETRACT", "R", "X", 0, 4,
                                          guard_auto=True)

        # ---- autonomous scan ----
        # One button instead of manually toggling FWD/BACK: click START and
        # the robot drives itself, stopping to examine cracks and running
        # the whole deploy/retract sequence on its own once one is
        # confirmed. Control is handed back to manual driving afterward.
        #   IDLE -> SCANNING -> PAUSED -> REPOSITIONING -> PRE_DEPLOY_WAIT
        #        -> DEPLOYING -> POST_DEPLOY_WAIT -> RETRACTING_AUTO -> IDLE
        # RETURNING_HOME is a separate path, reachable from IDLE via the
        # HOME button below, independent of the sequence above.
        self.auto_state = "IDLE"
        self._auto_confs = []
        self._auto_pause_ticks = 0
        self._reposition_start_mm = None
        self._wait_started_t = None       # PRE_/POST_DEPLOY_WAIT timers
        self._act_label = ""              # "deploying" / "retracting", for the status line
        self._act_last_val = None         # last seen telemetry "actuator" value
        self._act_last_change_t = None
        self._act_started_t = None
        self.start_btn = tk.Button(rob, text="▶ START", width=10,
                                   command=self._toggle_auto)
        self.start_btn.grid(row=0, column=5, padx=3, pady=3)
        self.home_btn = tk.Button(rob, text="⌂ RETURN HOME", width=13,
                                  command=self._on_return_home)
        self.home_btn.grid(row=0, column=6, padx=3, pady=3)
        self.lbl_auto = tk.Label(rob, text="AUTO: idle", bg="#1e1e1e",
                                 fg="#ccc", font=("Menlo", 11), anchor="w")
        self.lbl_auto.grid(row=4, column=0, columnspan=7,
                           sticky="w", padx=6, pady=(0, 4))

        # Servo + LED + zero: single click, latching.
        for col, (label, ch) in enumerate([
                ("HOOK", "H"), ("RELEASE", "U"), ("ASSEMBLY", "A"),
                ("LED ON", "L"), ("LED OFF", "O"), ("ZERO ODOM", "Z")]):
            tk.Button(rob, text=label, width=10,
                      command=lambda c=ch: self.link.send(c)
                      ).grid(row=1, column=col, padx=3, pady=3)

        # Lamp brightness. main.cpp maps the digits '0'-'9' onto 0-255, so a
        # single character carries the level and the protocol stays as it is.
        tk.Label(rob, text="lamp", bg="#1e1e1e", fg="#ccc").grid(
            row=2, column=0, sticky="e", padx=(6, 0))
        self.led_scale = tk.Scale(rob, from_=0, to=9, orient="horizontal",
                                  bg="#1e1e1e", fg="#ccc", highlightthickness=0,
                                  troughcolor="#333", length=200,
                                  command=self._on_led_scale)
        self.led_scale.set(4)
        self.led_scale.grid(row=2, column=1, columnspan=3, sticky="w")

        # Telemetry echo — confirms the S3 heard you. If you press LED ON and
        # nothing on this line changes, the command isn't reaching the board.
        self.lbl_telem = tk.Label(rob, text="telemetry: --", bg="#1e1e1e",
                                  fg="#8f8", font=("Menlo", 11), anchor="w")
        self.lbl_telem.grid(row=3, column=0, columnspan=6,
                            sticky="w", padx=6, pady=(2, 4))

        # ---- live diagnostics (so you never need to close the panel) ----
        diag = tk.LabelFrame(body, text="Diagnostics (live)",
                             bg="#1e1e1e", fg="#ccc")
        diag.pack(fill="x", padx=8, pady=4)
        self.lbl_diag = tk.Label(diag, text="", bg="#1e1e1e", fg="#8cf",
                                 font=("Menlo", 11), justify="left", anchor="w")
        self.lbl_diag.pack(fill="x", padx=6, pady=(4, 0))
        self.lbl_verdict = tk.Label(diag, text="", bg="#1e1e1e", fg="#0f0",
                                    font=("Menlo", 11, "bold"),
                                    justify="left", anchor="w")
        self.lbl_verdict.pack(fill="x", padx=6, pady=(0, 4))
        tk.Button(diag, text="PRINT FULL REPORT", command=self.print_report
                  ).pack(anchor="w", padx=6, pady=(0, 4))

        # ---- capture ----
        cap = tk.LabelFrame(body, text="Capture", bg="#1e1e1e", fg="#ccc")
        cap.pack(fill="x", padx=8, pady=4)
        self.rec_btn = tk.Button(cap, text="● REC", width=10, fg="#f55",
                                 command=self.toggle_record)
        self.rec_btn.grid(row=0, column=0, padx=4, pady=4)
        tk.Button(cap, text="SAVE still", width=11,
                  command=self.save_dataset).grid(row=0, column=1, padx=4)
        self.autocap = False
        self.btn_auto = tk.Button(cap, text="AUTO-CAP: OFF", width=13,
                                  command=self.toggle_autocap)
        self.btn_auto.grid(row=0, column=2, padx=4)
        tk.Button(cap, text="CALIB PAIR", width=11,
                  command=self.save_calib_pair).grid(row=0, column=3, padx=4)

        # ---- state ----
        self.recording = False
        self.writers = [None, None]
        self.rec_paths = None
        self.rec_job = None
        self._last_autocap = 0
        self._fps_t0 = time.time()
        self._start_time = time.time()   # for the diagnostics averages
        self._fps_base = (0, 0)
        for d in (REC_DIR, DATASET_DIR, CALIB_DIR):
            os.makedirs(d, exist_ok=True)

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._release_jobs = {}   # per-key debounce timers, see _key_up()
        self._bind_drive_keys(root)
        self._tick()

    # ---- recording ----
    def toggle_record(self):
        if not self.recording:
            ts = time.strftime("%Y%m%d_%H%M%S")
            self.rec_paths = [os.path.join(REC_DIR, f"left_{ts}.mp4"),
                              os.path.join(REC_DIR, f"right_{ts}.mp4")]
            self.writers = [None, None]
            self.recording = True
            self.rec_btn.config(text="■ STOP", bg="#8b0000", fg="white")
            self._rec_tick()
        else:
            self.recording = False
            if self.rec_job:
                self.root.after_cancel(self.rec_job)
                self.rec_job = None
            for w in self.writers:
                if w is not None:
                    w.release()
            self.writers = [None, None]
            self.rec_btn.config(text="● REC", bg="#d9d9d9", fg="#f55")
            print(f"saved -> {REC_DIR}")

    def _rec_tick(self):
        if not self.recording:
            return
        l, r, _ = self.link.get_pair()
        for i, img in enumerate((l, r)):
            if img is None:
                continue
            if self.writers[i] is None:
                h, w = img.shape[:2]
                self.writers[i] = cv2.VideoWriter(
                    self.rec_paths[i], cv2.VideoWriter_fourcc(*"mp4v"),
                    REC_FPS, (w, h))
            self.writers[i].write(img)
        self.rec_job = self.root.after(int(1000 / REC_FPS), self._rec_tick)

    # ---- stills ----
    def _save_pair(self, folder, tag):
        l, r, synced = self.link.get_pair()
        if l is None and r is None:
            print("no frames yet")
            return
        if l is not None and r is not None and not synced:
            print("WARNING: frames not synchronized — pair may be skewed")
        ts = int(time.time() * 1000)
        saved = []
        # Save whichever cameras are present — mono capture is fine for the
        # YOLO dataset; only stereo calibration needs true pairs.
        if l is not None:
            cv2.imwrite(os.path.join(folder, f"{tag}_{ts}_L.png"), l)
            saved.append("L")
        if r is not None:
            cv2.imwrite(os.path.join(folder, f"{tag}_{ts}_R.png"), r)
            saved.append("R")
        note = "" if len(saved) == 2 else "  (single camera)"
        print(f"saved {tag}_{ts} [{'+'.join(saved)}] -> {folder}{note}")

    def save_dataset(self):
        self._save_pair(DATASET_DIR, "data")

    def save_calib_pair(self):
        self._save_pair(CALIB_DIR, "calib")

    def toggle_autocap(self):
        self.autocap = not self.autocap
        self.btn_auto.config(text=f"AUTO-CAP: {'ON' if self.autocap else 'OFF'}")

    # ---- robot control plumbing ----
    def _hold_btn(self, parent, text, cmd, stop_cmd, r, c,
                  on_press=None, on_release=None, guard_auto=False):
        """Press-and-hold button: sends `cmd` on press, `stop_cmd` on release.

        on_press/on_release, if given, run after the corresponding command
        is sent. guard_auto=True makes the press a no-op while the
        autonomous sequence is running (auto_state != "IDLE") -- used for
        DEPLOY/RETRACT so a manual click can't collide with the automatic
        deploy/retract steps of that sequence. Release always still sends
        stop_cmd regardless, since that's a safe no-op if press was ignored.
        """
        b = tk.Button(parent, text=text, width=10)
        b.grid(row=r, column=c, padx=3, pady=3)

        def _press(e):
            if guard_auto and self.auto_state != "IDLE":
                print(f"AUTO sequence running -- ignoring manual {text} click")
                return
            self.link.send(cmd)
            if on_press:
                on_press()

        def _release(e):
            self.link.send(stop_cmd)
            if on_release:
                on_release()
        b.bind("<ButtonPress-1>", _press)
        b.bind("<ButtonRelease-1>", _release)
        # If the window loses focus mid-press we'd never see the release, so
        # stop on leave too — a motor left running is worse than a jerky UI.
        b.bind("<Leave>", lambda e: self.link.send(stop_cmd))
        return b

    def _toggle_forward(self):
        if self.auto_state != "IDLE":
            print("AUTO mode is running -- click START to stop it before driving manually")
            return
        if self.drive_state == 1:
            self.link.send("S")
            self.drive_state = 0
        else:
            self.link.send("F")
            self.drive_state = 1
        self._update_drive_buttons()

    def _toggle_backward(self):
        if self.auto_state != "IDLE":
            print("AUTO mode is running -- click START to stop it before driving manually")
            return
        if self.drive_state == -1:
            self.link.send("S")
            self.drive_state = 0
        else:
            self.link.send("B")
            self.drive_state = -1
        self._update_drive_buttons()

    def _update_drive_buttons(self):
        # Highlight whichever direction is currently active so it's obvious
        # at a glance the robot is still moving without watching the video.
        self.fwd_btn.config(bg="#2d6b2d" if self.drive_state == 1 else "#d9d9d9",
                            fg="white" if self.drive_state == 1 else "black")
        self.back_btn.config(bg="#2d6b2d" if self.drive_state == -1 else "#d9d9d9",
                             fg="white" if self.drive_state == -1 else "black")

    # ---- autonomous scan loop ----
    # States: IDLE -> SCANNING -> PAUSED -> REPOSITIONING -> PRE_DEPLOY_WAIT
    #      -> DEPLOYING -> POST_DEPLOY_WAIT -> RETRACTING_AUTO -> IDLE
    # RETURNING_HOME is a separate path, reachable only from IDLE via the
    # RETURN HOME button, independent of the sequence above.
    # See _auto_step() for the per-tick logic; this block is just the
    # entry/exit points reachable from buttons.

    def _toggle_auto(self):
        if self.auto_state == "IDLE":
            self.auto_state = "SCANNING"
            self._auto_confs = []
            self._auto_pause_ticks = 0
            self.link.send("F")
            self.drive_state = 1
            self._update_drive_buttons()
            self.start_btn.config(text="■ STOP SCAN", bg="#2d6b2d", fg="white")
        else:
            self._cancel_auto()

    def _cancel_auto(self):
        self.auto_state = "IDLE"
        self.link.send("S")   # stop drive
        self.link.send("X")   # stop actuator, in case this cancelled mid deploy/retract
        self.drive_state = 0
        self._update_drive_buttons()
        self.start_btn.config(text="▶ START", bg="#d9d9d9", fg="black")
        self.lbl_auto.config(text="AUTO: idle", fg="#ccc")

    def _current_pos_mm(self):
        """Best-effort read of how far the robot has driven, per telemetry."""
        with self.link.lock:
            t = self.link.telem
            if "pos_mm" in t:
                return float(t["pos_mm"])
            if "steps" in t:
                return float(t["steps"]) / STEPS_PER_MM_FALLBACK
        return None

    def _actuator_settled(self, now):
        """True once telemetry's "actuator" field has stopped changing for
        ACTUATOR_STABLE_SECS (or a safety timeout / no-telemetry fallback
        has elapsed) -- used to detect "fully deployed" and "fully
        retracted" without needing to know what the numbers actually mean.
        Also updates the status label. Caller must have set
        self._act_label/_act_last_val/_act_last_change_t/_act_started_t
        when entering the watching state."""
        with self.link.lock:
            val = self.link.telem.get("actuator")
        if val is None:
            elapsed = now - self._act_started_t
            self.lbl_auto.config(
                text=f"AUTO: {self._act_label} (no actuator telemetry, {elapsed:.1f}s)",
                fg="#e8a33d")
            return elapsed >= ACTUATOR_NO_TELEM_WAIT_SECS
        if self._act_last_val is None or val != self._act_last_val:
            self._act_last_val = val
            self._act_last_change_t = now
        stable_for = now - self._act_last_change_t
        self.lbl_auto.config(
            text=f"AUTO: {self._act_label} (actuator={val}, stable {stable_for:.1f}s)",
            fg="#8cf")
        timed_out = (now - self._act_started_t) >= ACTUATOR_TIMEOUT_SECS
        return stable_for >= ACTUATOR_STABLE_SECS or timed_out

    def _auto_step(self, detections, now):
        """Runs once per _tick(). `detections` is this frame's combined
        L+R confirmed-crack list. Advances the state machine below."""
        if self.auto_state == "IDLE":
            return

        if self.auto_state == "SCANNING":
            if self.drive_state != 1:
                self.link.send("F")
                self.drive_state = 1
                self._update_drive_buttons()
            if any(d.conf >= AUTO_PAUSE_TRIGGER_CONF for d in detections):
                # Stop immediately for a clearer, non-motion-blurred look
                # before trusting any confidence number.
                self.link.send("S")
                self.drive_state = 0
                self._update_drive_buttons()
                self.auto_state = "PAUSED"
                self._auto_pause_ticks = 0
                self._auto_confs = []
            self.lbl_auto.config(text="AUTO: scanning...", fg="#0f0")
            return

        if self.auto_state == "PAUSED":
            self._auto_pause_ticks += 1
            if self._auto_pause_ticks <= AUTO_PAUSE_SETTLE_TICKS:
                self.lbl_auto.config(text="AUTO: stopped, settling...", fg="#e8a33d")
                return
            self._auto_confs.append(max((d.conf for d in detections), default=0.0))
            avg = sum(self._auto_confs) / len(self._auto_confs)
            self.lbl_auto.config(
                text=f"AUTO: checking crack — conf {avg:.2f} "
                     f"({len(self._auto_confs)}/{AUTO_PAUSE_CONFIRM_TICKS})",
                fg="#e8a33d")
            if len(self._auto_confs) >= AUTO_PAUSE_CONFIRM_TICKS:
                if avg >= AUTO_DEPLOY_CONF:
                    self.auto_state = "REPOSITIONING"
                    self._reposition_start_mm = self._current_pos_mm()
                    self.link.send("F")
                    self.drive_state = 1
                    self._update_drive_buttons()
                else:
                    # False alarm (shadow/seam) -- resume scanning forward.
                    self.auto_state = "SCANNING"
                    self.link.send("F")
                    self.drive_state = 1
                    self._update_drive_buttons()
            return

        if self.auto_state == "REPOSITIONING":
            pos = self._current_pos_mm()
            if self._reposition_start_mm is None or pos is None:
                # No usable telemetry -- can't measure the reposition
                # distance, so bail safely to a stop and proceed anyway
                # rather than drive forward blind indefinitely.
                self.link.send("S")
                self.drive_state = 0
                self._update_drive_buttons()
                self.auto_state = "PRE_DEPLOY_WAIT"
                self._wait_started_t = now
                self.lbl_auto.config(
                    text="AUTO: no telemetry -- stopped early, waiting to deploy", fg="#f55")
                return
            traveled = abs(pos - self._reposition_start_mm)
            self.lbl_auto.config(
                text=f"AUTO: repositioning {traveled:.0f}/{AUTO_REPOSITION_MM:.0f} mm",
                fg="#8cf")
            if traveled >= AUTO_REPOSITION_MM:
                self.link.send("S")
                self.drive_state = 0
                self._update_drive_buttons()
                self.auto_state = "PRE_DEPLOY_WAIT"
                self._wait_started_t = now
            return

        if self.auto_state == "PRE_DEPLOY_WAIT":
            elapsed = now - self._wait_started_t
            remaining = max(0.0, PRE_DEPLOY_WAIT_SECS - elapsed)
            self.lbl_auto.config(
                text=f"AUTO: crack confirmed -- deploying in {remaining:.1f}s", fg="#e8a33d")
            if elapsed >= PRE_DEPLOY_WAIT_SECS:
                self.auto_state = "DEPLOYING"
                self.link.send("D")
                self._act_label = "deploying"
                self._act_last_val = None
                self._act_last_change_t = now
                self._act_started_t = now
            return

        if self.auto_state == "DEPLOYING":
            if self._actuator_settled(now):
                self.auto_state = "POST_DEPLOY_WAIT"
                self._wait_started_t = now
            return

        if self.auto_state == "POST_DEPLOY_WAIT":
            elapsed = now - self._wait_started_t
            remaining = max(0.0, POST_DEPLOY_WAIT_SECS - elapsed)
            self.lbl_auto.config(
                text=f"AUTO: deployed -- retracting in {remaining:.1f}s", fg="#e8a33d")
            if elapsed >= POST_DEPLOY_WAIT_SECS:
                self.auto_state = "RETRACTING_AUTO"
                self.link.send("R")
                self._act_label = "retracting"
                self._act_last_val = None
                self._act_last_change_t = now
                self._act_started_t = now
            return

        if self.auto_state == "RETRACTING_AUTO":
            if self._actuator_settled(now):
                self.link.send("X")   # explicit stop, safe even if already done
                self.auto_state = "IDLE"
                self.start_btn.config(text="▶ START", bg="#d9d9d9", fg="black")
                self.lbl_auto.config(
                    text="AUTO: sequence complete -- manual control (or RETURN HOME)",
                    fg="#ccc")
            return

        if self.auto_state == "RETURNING_HOME":
            # Deliberately ignores `detections` entirely -- once homing has
            # started, it must not stop for a crack. Only telemetry loss or
            # an explicit cancel (STOP ALL / clicking the button again) can
            # interrupt it before it reaches home.
            pos = self._current_pos_mm()
            if pos is None:
                self.link.send("S")
                self.drive_state = 0
                self._update_drive_buttons()
                self.auto_state = "IDLE"
                self.start_btn.config(text="▶ START", bg="#d9d9d9", fg="black")
                self.lbl_auto.config(
                    text="AUTO: no telemetry -- can't return home, stopped", fg="#f55")
                return
            diff = pos - 0.0   # "home" = wherever ZERO ODOM was last pressed
            if abs(diff) <= HOME_TOLERANCE_MM:
                self.link.send("S")
                self.drive_state = 0
                self._update_drive_buttons()
                self.auto_state = "IDLE"
                self.start_btn.config(text="▶ START", bg="#d9d9d9", fg="black")
                self.lbl_auto.config(
                    text="AUTO: home -- click START to scan again", fg="#ccc")
                return
            # Direction-aware and re-checked every tick, so it self-corrects
            # even if it overshoots or starts on either side of home.
            desired = -1 if diff > 0 else 1
            if self.drive_state != desired:
                self.link.send("B" if desired == -1 else "F")
                self.drive_state = desired
                self._update_drive_buttons()
            self.lbl_auto.config(
                text=f"AUTO: returning home ({abs(diff):.0f} mm left)", fg="#8cf")
            return

    def _on_return_home(self):
        """RETURN HOME button -- only available when nothing else is
        running (a crack sequence must finish or be cancelled first)."""
        if self.auto_state != "IDLE":
            print("AUTO sequence running -- finish or STOP ALL before returning home")
            return
        if self._current_pos_mm() is None:
            print("No telemetry -- can't return home without position feedback")
            return
        self.auto_state = "RETURNING_HOME"
        self.start_btn.config(text="■ STOP", bg="#2d6b2d", fg="white")
        self.lbl_auto.config(text="AUTO: returning home...", fg="#8cf")


    # ---- keyboard drive: F/B keys, works ALONGSIDE the click-toggle
    # buttons above (both just flip the same drive_state) ----
    #
    # Holding a key down on any OS does NOT deliver one continuous "key is
    # down" signal to Tkinter -- the OS's own key-repeat fires a rapid
    # alternating stream of KeyRelease/KeyPress events for as long as you
    # hold it. A naive "release = stop" binding would therefore stutter:
    # stop, restart, stop, restart, many times a second while held.
    #
    # Fix: on release, don't stop immediately -- schedule the stop after a
    # short delay. If the key is actually still held, the next auto-repeat
    # KeyPress arrives within that delay and cancels the pending stop. Only
    # a GENUINE release (nothing arrives in time) lets the stop go through.
    # A quick tap also works correctly: press moves, and since no repeat
    # follows, the debounced stop fires almost immediately after.
    _KEY_DEBOUNCE_MS = 60

    def _bind_drive_keys(self, root):
        root.bind("<KeyPress-f>", lambda e: self._key_down("f"))
        root.bind("<KeyRelease-f>", lambda e: self._key_up("f"))
        root.bind("<KeyPress-b>", lambda e: self._key_down("b"))
        root.bind("<KeyRelease-b>", lambda e: self._key_up("b"))
        root.focus_set()   # window must have keyboard focus to see these at all

    def _key_down(self, key):
        if self.auto_state != "IDLE":
            return   # AUTO mode owns the drive commands right now
        pending = self._release_jobs.pop(key, None)
        if pending is not None:
            self.root.after_cancel(pending)
            return   # this was an auto-repeat press -- already moving, ignore
        if key == "f" and self.drive_state != 1:
            self.link.send("F")
            self.drive_state = 1
            self._update_drive_buttons()
        elif key == "b" and self.drive_state != -1:
            self.link.send("B")
            self.drive_state = -1
            self._update_drive_buttons()

    def _key_up(self, key):
        self._release_jobs[key] = self.root.after(
            self._KEY_DEBOUNCE_MS, lambda: self._key_up_confirmed(key))

    def _key_up_confirmed(self, key):
        self._release_jobs.pop(key, None)
        if (key == "f" and self.drive_state == 1) or \
           (key == "b" and self.drive_state == -1):
            self.link.send("S")
            self.drive_state = 0
            self._update_drive_buttons()

    def _on_led_scale(self, val):
        # Tk fires this for every pixel of drag, so only send on change —
        # otherwise a single sweep floods the port with dozens of characters
        # and competes with the video for bandwidth.
        v = int(float(val))
        if v != getattr(self, "_led_last", None):
            self._led_last = v
            self.link.send(str(v))

    def _on_conf_scale(self, val):
        v = float(val)
        if self.cv_tracker_l is not None:
            self.cv_tracker_l.conf_threshold = v
        if self.cv_tracker_r is not None:
            self.cv_tracker_r.conf_threshold = v

    def stop_all(self):
        """Panic button: stop drive AND actuator, and cancel AUTO mode."""
        self.link.send("S")
        self.link.send("X")
        self.drive_state = 0
        self._update_drive_buttons()
        if self.auto_state != "IDLE":
            self._cancel_auto()

    # ---- live diagnostics ----
    def _diag_numbers(self):
        """Everything stream_quality.py reports, computed from the live
        stream. Returns (text_lines, verdict, verdict_colour)."""
        L = self.link
        run = max(0.1, time.time() - self._start_time)
        good = L.counts[0] + L.counts[1]
        bad = L.dropped
        total = good + bad
        pct_bad = 100 * bad / total if total else 0.0
        thru = L.bytes_total / run
        unacc = max(0, L.bytes_total - L.bytes_frames)
        pct_unacc = 100 * unacc / L.bytes_total if L.bytes_total else 0.0

        lines = [
            f"throughput  {thru:7.0f} B/s   unparsed {pct_unacc:3.0f}%"
            f"   telemetry {L.telem_count}",
            f"frames      good {good}   bad {bad} ({pct_bad:.0f}%)"
            f"   L{L.counts[0]} / R{L.counts[1]}",
            f"drops       badstart {L.drop_badstart}  spliced {L.drop_spliced}"
            f"  nodecode {L.drop_decode}  truncated {L.drop_flat}"
            f"  colourband {L.drop_band}",
        ]

        with L.lock:
            s = dict(L.last_stats)
        if 0 in s and 1 in s:
            (bl, cl_), (br, cr_) = s[0], s[1]
            d = 100 * abs(bl - br) / max(bl, br, 1e-6)
            lines.append(f"exposure    L bright {bl:3.0f} / R {br:3.0f}"
                         f"   diff {d:3.0f}%   (L con {cl_:.0f} / R {cr_:.0f})")
        else:
            d = None

        # Colour drift: R/G/B now, and how far each has moved since the first
        # frame. "Purple/magenta" means G has fallen relative to R and B, so
        # the signed deltas say whether green is dying or red+blue are rising.
        with L.lock:
            cs, cref, ct0 = dict(L.chan_stats), dict(L.chan_ref), dict(L.chan_t0)
        for cam, tag in ((0, "L"), (1, "R")):
            cur = cs.get(cam)
            if cur is None:
                continue
            r, g, b = cur
            # Green deficit vs the average of red and blue. Positive = magenta.
            cast = (r + b) / 2 - g
            txt = (f"colour {tag}    R{r:5.1f} G{g:5.1f} B{b:5.1f}"
                   f"   magenta {cast:+5.1f}")
            ref = cref.get(cam)
            if ref and ct0.get(cam):
                dr, dg, db = (r - ref[0], g - ref[1], b - ref[2])
                mins = (time.time() - ct0[cam]) / 60.0
                txt += (f"   drift {dr:+5.1f}/{dg:+5.1f}/{db:+5.1f}"
                        f" over {mins:4.1f} min")
            lines.append(txt)

        # Verdict — the same judgements I'd make reading a report.
        if L.port_error:
            return lines, f"PORT DIED: {L.port_error}", "#f55"
        if total == 0:
            return lines, ("NO FRAMES. S3 running? camera<->S3 baud match? "
                           "check camera LEDs"), "#f55"
        # Truncation must be judged BEFORE "camera silent": if every frame is
        # truncated, the accepted count is zero and it would look like a dead
        # camera when really the frames are arriving and being rejected.
        if L.drop_flat > 0.2 * total:
            return lines, ("TRUNCATED FRAMES — bytes lost before the S3 "
                           "forwarded them (wire/power)"), "#e8a33d"
        if L.counts[0] == 0 or L.counts[1] == 0:
            miss = "LEFT" if L.counts[0] == 0 else "RIGHT"
            return lines, f"{miss} camera silent — its 5V/GND/U0T, or sensor init", "#f55"
        if pct_unacc > 20:
            return lines, "STREAM DESYNC — most bytes aren't frames", "#f55"
        if pct_bad > 10:
            return lines, f"{pct_bad:.0f}% of frames rejected — link is marginal", "#e8a33d"
        if d is not None and d > 40:
            return lines, ("CAMERAS DISAGREE on exposure — same firmware on "
                           "both? auto-exposure off?"), "#e8a33d"
        return lines, "healthy", "#0f0"

    def _update_diagnostics(self, fl, fr, now):
        lines, verdict, colour = self._diag_numbers()
        self.lbl_diag.config(text="\n".join(lines))
        self.lbl_verdict.config(text="-> " + verdict, fg=colour)

    def print_report(self):
        """Dump a full snapshot to the terminal — the stream_quality report,
        but without having to close the panel to free the port."""
        lines, verdict, _ = self._diag_numbers()
        run = time.time() - self._start_time
        print("\n" + "=" * 58)
        print(f"PIPE CAM DIAGNOSTIC REPORT   ({run:.0f}s of streaming)")
        print("=" * 58)
        for l in lines:
            print("  " + l)
        t = self.link.telem
        if t:
            print("  telemetry   " + ", ".join(f"{k}={v}" for k, v in t.items()))
        print(f"\n  -> {verdict}")
        print("=" * 58 + "\n")

    def _draw_detections(self, img, detections, cv_lines, tag):
        """Draws confirmed detections onto img in place, and appends their
        text summary (with tag "L"/"R" so the panel shows which camera each
        came from) into cv_lines. No distance here -- camera-distance was
        removed as unreliable; DISTANCE TRAVELED (stepper odometry) is the
        only distance this panel reports."""
        for det in detections:
            pts = det.mask_xy
            cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255),
                         thickness=2)
            label_txt = f"{det.label} {det.conf:.2f}"
            (tw_, th_), _ = cv2.getTextSize(label_txt,
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            x1, y1 = pts[:, 0].min(), pts[:, 1].min()
            cv2.rectangle(img, (x1, y1 - th_ - 6), (x1 + tw_ + 4, y1),
                         (0, 0, 0), -1)
            cv2.putText(img, label_txt, (x1 + 2, y1 - 4),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1,
                       cv2.LINE_AA)
            cv_lines.append(f"{tag} ID{det.track_id}  {det.label:<6}"
                           f"  conf {det.conf:.2f}")

    # ---- main loop ----
    def _tick(self):
        img_l, img_r = self.worker.snapshot()

        det_l, det_r = [], []
        if img_l is not None and self.cv_tracker_l is not None:
            try:
                det_l = self.cv_tracker_l.update(img_l)
            except Exception as e:
                self.lbl_cv.config(text=f"CV error (L): {e}", fg="#f55")
        if img_r is not None and self.cv_tracker_r is not None:
            try:
                det_r = self.cv_tracker_r.update(img_r)
            except Exception as e:
                self.lbl_cv.config(text=f"CV error (R): {e}", fg="#f55")

        cv_lines = []

        if img_l is not None:
            vis_l = img_l.copy()
            self._draw_detections(vis_l, det_l, cv_lines, "L")
            self._show(self.lblL, vis_l)

        if img_r is not None:
            vis_r = img_r.copy()
            self._draw_detections(vis_r, det_r, cv_lines, "R")
            self._show(self.lblR, vis_r)

        if (self.cv_tracker_l is not None or self.cv_tracker_r is not None) \
                and not self.cv_error:
            self.lbl_cv.config(
                text="\n".join(cv_lines) if cv_lines else "no confirmed detections",
                fg="#f66")

        now = time.time()
        self._auto_step(det_l + det_r, now)

        # fps + health line
        if now - self._fps_t0 >= 1.0:
            cl, cr = self.link.counts[0], self.link.counts[1]
            fl = (cl - self._fps_base[0]) / (now - self._fps_t0)
            fr = (cr - self._fps_base[1]) / (now - self._fps_t0)
            self._fps_base = (cl, cr)
            self._fps_t0 = now
            msg = f"L {fl:.1f} fps   R {fr:.1f} fps   dropped {self.link.dropped}"
            if self.recording:
                msg += "   ● RECORDING"
            self.lbl_status.config(text=msg, fg="#0f0")
            self._update_diagnostics(fl, fr, now)

            with self.link.lock:
                t = dict(self.link.telem)
            # IMU-derived fields aren't trusted, so they're dropped from
            # display here even if the firmware still sends them.
            t = {k: v for k, v in t.items()
                if k not in ("imu_pos", "imu_vel", "imu_bias", "slip")}
            if t:
                order = ["servo_angle", "steps", "stepper_drive_dir", "actuator"]
                keys = [k for k in order if k in t] + \
                       [k for k in t if k not in order]
                self.lbl_telem.config(
                    text="  ".join(f"{k}={t[k]}" for k in keys), fg="#8f8")
                if "pos_mm" in t:
                    self.lbl_traveled.config(
                        text=f"DISTANCE TRAVELED: {t['pos_mm'] / 10:.1f} cm")
                elif "steps" in t:
                    # Older firmware without pos_mm yet -- show raw steps
                    # rather than nothing, so this still means something.
                    self.lbl_traveled.config(
                        text=f"DISTANCE TRAVELED: {t['steps']} steps (raw)")
            else:
                self.lbl_telem.config(
                    text="telemetry: none received — S3 not sending?", fg="#e8a33d")

        if self.autocap and now - self._last_autocap > AUTOCAP_SECS:
            self.save_dataset()
            self._last_autocap = now

        self.root.after(40, self._tick)

    def _show(self, lbl, bgr):
        img = Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        if SCALE != 1:
            img = img.resize((img.width * SCALE, img.height * SCALE),
                             Image.NEAREST)
        photo = ImageTk.PhotoImage(img)
        lbl.configure(image=photo, text="", width=img.width, height=img.height)
        lbl.image = photo          # keep a reference or Tk garbage-collects it

    def on_close(self):
        if self.recording:
            self.toggle_record()   # flush and close the MP4s
        self.worker.running = False
        self.link.close()
        try:
            self.link.ser.close()
        except serial.SerialException:
            pass
        self.root.destroy()


# ============================ MAIN ============================
def main():
    port = find_port()
    if port is None:
        print("No ESP32 serial port found. Is the S3 plugged in via USB?")
        return
    try:
        ser = serial.Serial(port, BAUD, timeout=0.05)
    except serial.SerialException as e:
        print(f"Could not open {port}: {e}")
        print("If busy: close the Arduino Serial Monitor or another panel.")
        return
    print("Using port:", port)

    link = FrameLink(ser)

    # Record per-channel colour every frame. It's a few hundred KB an hour and
    # it means a drift complaint can be answered with a graph instead of a
    # memory of what the screen looked like ten minutes ago.
    os.makedirs(DATA_DIR, exist_ok=True)
    log_path = os.path.join(DATA_DIR, "colour_drift.csv")
    link.chan_log = open(log_path, "w", buffering=1)
    link.chan_log.write("t,cam,r,g,b\n")
    print("logging colour drift to", log_path)

    root = tk.Tk()
    App(root, link)
    root.mainloop()


if __name__ == "__main__":
    main()