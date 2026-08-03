"""
control_panel_stereo.py — EDI PIPE CAM control panel (hardware rev 3)
=====================================================================
Single laptop-side program. Talks to the ESP32-S3 over one USB serial
port; the S3 forwards both camera streams plus telemetry.

Features
  * Live left/right video, 2x scaled
  * Telemetry: stepper odometry (mm), IMU pitch/roll, drive/actuator/servo
  * Crack detection (classical edge heuristic, or YOLO if you have weights)
  * Distance to each detected crack via stereo matching
  * Servo controls (HOOK / RELEASE / ASSEMBLY) and ZERO ODOM
  * Lining-pull workflow: mark start, enter crack length, drive back until
    the pulled length covers it (turns green)
  * REC: record both feeds to MP4 (for building the training set)
  * CALIB PAIR / SAVE / AUTO-CAP: save synchronized PNG stills

Frame protocol from the S3 over USB CDC:
    0xAA 0x55 | id(1) | len(4 LE) | payload
    id 0 = LEFT JPEG, 1 = RIGHT JPEG, 2 = telemetry JSON
(The bench ids b'L'/b'R' are also accepted — see stereo_serial.py.)

Why stepper odometry, not IMU distance: each step is a fixed travel
increment (exact, drift-free); double-integrating IMU acceleration drifts
quadratically and is unusable after ~1 s. The IMU is for roll (crack clock
position) and slip detection.

Run:
    source ~/Downloads/esp32env/bin/activate
    cd ~/Desktop/30.007/"camera codes"
    python3 control_panel_stereo.py

Dependencies:  pip install pyserial opencv-python pillow numpy
               (+ pip install ultralytics   -- only if MODE = "yolo")
"""

import json
import os
import queue
import threading
import time
import tkinter as tk

import cv2
import numpy as np
import serial
import serial.tools.list_ports
from PIL import Image, ImageTk

# ============================ CONFIG ============================
HERE = os.path.dirname(os.path.abspath(__file__))

PORT          = None            # None = auto-detect. Or "/dev/cu.usbmodem101"
BAUD          = 921600          # ignored by USB CDC, but pyserial wants it
MODE          = "classical"     # "none" | "classical" | "yolo"
YOLO_WEIGHTS  = os.path.join(HERE, "best.pt")
CALIB_FILE    = os.path.join(HERE, "stereo_calib.npz")
CMD_REPEAT_MS = 150             # must be < firmware WATCHDOG_MS (400)
DETECT_EVERY  = 0.25            # seconds between detection passes
PAIR_MAX_AGE  = 0.30            # L/R frames further apart than this = unsynced
SCALE         = 2               # display zoom (320x240 -> 640x480)
PULL_MARGIN_CM = 4.0            # overlap added to entered crack length

REC_FPS       = 10
REC_DIR       = os.path.expanduser("~/Downloads/pipe_cam_recordings")
CALIB_DIR     = os.path.expanduser("~/Downloads/pipe_cam_calib_pairs")
DATASET_DIR   = os.path.expanduser("~/Downloads/pipe_cam_dataset")
AUTOCAP_SECS  = 1.0

# Camera command escapes relayed by the S3: "@" + target + cmd
CAM_CMDS = [("quality +", "@aQ"), ("quality -", "@aq"),
            ("bright +", "@aB"), ("bright -", "@ab"),
            ("LED on", "@aF"), ("LED off", "@af")]


# ============================ SERIAL ============================
def find_port():
    """Return PORT if set, else the most likely ESP32 serial device."""
    if PORT:
        return PORT
    cands = [p.device for p in serial.tools.list_ports.comports()
             if any(k in p.device.lower()
                    for k in ("usbmodem", "usbserial", "ttyacm", "ttyusb"))]
    if not cands:
        return None
    if len(cands) > 1:
        print("Multiple serial devices found:", ", ".join(cands))
        print(f"Using {cands[0]} — set PORT at the top to override.")
    return cands[0]


class FrameLink:
    """Reader thread: parses AA55 frames off the wire and hands JPEG bytes
    to a decoder thread.

    The reader NEVER decodes — JPEG decoding takes long enough that the USB
    buffer overflows while it runs, which shows up as torn frames.
    """

    MAGIC = b"\xAA\x55"
    ID_MAP = {b"\x00": 0, b"\x01": 1, b"L": 0, b"R": 1}
    TELEM_ID = b"\x02"
    MAX_FRAME = 300_000

    def __init__(self, ser):
        self.ser = ser
        self.latest = {0: None, 1: None}     # cam id -> (bgr image, timestamp)
        self.telem = {}
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

    def _read_exact(self, n):
        buf = bytearray()
        while self.running and len(buf) < n:
            chunk = self.ser.read(n - len(buf))
            if chunk:
                buf.extend(chunk)
        return bytes(buf)

    def _reader(self):
        sync = bytearray()
        while self.running:
            b = self.ser.read(1)
            if not b:
                continue
            sync += b
            if len(sync) > 2:
                del sync[0]
            if bytes(sync) != self.MAGIC:
                continue
            sync.clear()

            head = self._read_exact(5)
            if len(head) < 5:
                continue
            raw_id = head[:1]
            length = int.from_bytes(head[1:5], "little")
            if not (0 < length <= self.MAX_FRAME):
                continue                       # garbage -> resync

            # Telemetry must be consumed, not skipped, or its bytes get
            # mistaken for image data.
            if raw_id == self.TELEM_ID:
                body = self._read_exact(length)
                try:
                    t = json.loads(body.decode("ascii", "ignore"))
                    with self.lock:
                        self.telem = t
                except json.JSONDecodeError:
                    pass
                continue

            cam = self.ID_MAP.get(raw_id)
            if cam is None:
                continue

            payload = self._read_exact(length)
            if len(payload) != length:
                continue

            # Validate + trim: the sensor pads bytes after the JPEG end
            # marker, and torn frames render as gray-bottom garbage.
            if payload[:2] != b"\xFF\xD8":
                continue
            end = payload.rfind(b"\xFF\xD9")
            if end < 0:
                continue

            try:
                self._jpegq.put_nowait((cam, payload[:end + 2], time.time()))
            except queue.Full:
                pass                            # decoder busy; drop this frame

    def _decoder(self):
        while self.running:
            try:
                cam, jpg, ts = self._jpegq.get(timeout=0.2)
            except queue.Empty:
                continue
            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                with self.lock:
                    self.latest[cam] = (img, ts)

    def get_pair(self):
        """Returns (left, right, synced). synced=False means the two frames
        are too far apart in time to trust for distance."""
        with self.lock:
            l, r = self.latest[0], self.latest[1]
        if l is None or r is None:
            return (l[0] if l else None), (r[0] if r else None), False
        return l[0], r[0], abs(l[1] - r[1]) <= PAIR_MAX_AGE

    def get_telem(self):
        with self.lock:
            return dict(self.telem)

    def close(self):
        self.running = False


# ============================ CALIBRATION ============================
class StereoCalib:
    def __init__(self, path):
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
            self.baseline = float(d["baseline"])   # mm
            self.ok = True
        except KeyError:
            # Old-format file from an earlier calibration run.
            self.error = ("stereo_calib.npz is an old format — "
                          "re-run stereo_calibrate.py")

    def rectify(self, left, right):
        if not self.ok:
            return left, right
        return (cv2.remap(left, self.map1x, self.map1y, cv2.INTER_LINEAR),
                cv2.remap(right, self.map2x, self.map2y, cv2.INTER_LINEAR))

    def depth_mm(self, disparity_px):
        if not self.ok or disparity_px is None or disparity_px <= 0.5:
            return None
        return self.fx * self.baseline / disparity_px


# ============================ DETECTORS ============================
def detect_classical(bgr):
    """Cheap placeholder: finds long thin dark features. Expect false
    positives on textured pipe — swap to YOLO once you have weights."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    th = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                               cv2.THRESH_BINARY_INV, 31, 8)
    th = cv2.morphologyEx(th, cv2.MORPH_CLOSE,
                          cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
    boxes = []
    cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        area = cv2.contourArea(c)
        if area < 60:
            continue
        x, y, w, h = cv2.boundingRect(c)
        elong = max(w, h) / max(1, min(w, h))
        fill = area / max(1, w * h)
        if elong > 2.5 and fill < 0.5:
            boxes.append((x, y, w, h))
    return boxes


class YoloDetector:
    def __init__(self, weights):
        from ultralytics import YOLO
        self.model = YOLO(weights)

    def __call__(self, bgr):
        res = self.model.predict(bgr, imgsz=320, conf=0.4, verbose=False)[0]
        return [(int(b[0]), int(b[1]), int(b[2] - b[0]), int(b[3] - b[1]))
                for b in res.boxes.xyxy.cpu().numpy()]


# ============================ STEREO MATCH ============================
def match_disparity(left_rect, right_rect, box):
    """Find the box's match along the same row band in the right image.
    Rectified images make this a 1-D search."""
    x, y, w, h = box
    H, W = left_rect.shape[:2]
    x, y = max(0, x), max(0, y)
    w, h = min(w, W - x), min(h, H - y)
    if w < 8 or h < 8:
        return None
    tmpl = cv2.cvtColor(left_rect[y:y + h, x:x + w], cv2.COLOR_BGR2GRAY)
    band = right_rect[max(0, y - 4):min(H, y + h + 4), :]
    strip = cv2.cvtColor(band, cv2.COLOR_BGR2GRAY)
    if strip.shape[0] < tmpl.shape[0] or strip.shape[1] < tmpl.shape[1]:
        return None
    res = cv2.matchTemplate(strip, tmpl, cv2.TM_CCOEFF_NORMED)
    _, maxv, _, maxloc = cv2.minMaxLoc(res)
    if maxv < 0.5:
        return None
    d = float(x - maxloc[0])
    return d if d > 0.5 else None


# ============================ CV WORKER ============================
class Detector(threading.Thread):
    def __init__(self, link, calib):
        super().__init__(daemon=True)
        self.link, self.calib = link, calib
        self.result = None
        self.lock = threading.Lock()
        self.yolo = None
        if MODE == "yolo":
            try:
                self.yolo = YoloDetector(YOLO_WEIGHTS)
            except Exception as e:
                print(f"YOLO unavailable ({e}) — falling back to classical")
        self.running = True
        self.start()

    def run(self):
        while self.running:
            time.sleep(DETECT_EVERY)
            if MODE == "none":
                continue
            l, r, synced = self.link.get_pair()
            if l is None or r is None:
                continue
            lr, rr = self.calib.rectify(l.copy(), r.copy())
            try:
                boxes = self.yolo(lr) if self.yolo else detect_classical(lr)
            except Exception:
                boxes = []
            info = f"{len(boxes)} crack(s)"
            if not synced:
                info += "  (unsynced — distances unreliable)"
            for box in boxes:
                x, y, w, h = box
                cv2.rectangle(lr, (x, y), (x + w, y + h), (0, 0, 255), 2)
                label = ""
                if not self.calib.ok:
                    label = "no calib"
                elif synced:
                    d = match_disparity(lr, rr, box)
                    z = self.calib.depth_mm(d)
                    if z is not None:
                        label = f"{z / 10:.1f} cm"
                        cv2.rectangle(rr, (int(x - d), y),
                                      (int(x - d + w), y + h), (0, 255, 255), 1)
                if label:
                    cv2.putText(lr, label, (x, max(12, y - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            with self.lock:
                self.result = (lr, rr, info)

    def get(self):
        with self.lock:
            return self.result


# ============================ GUI ============================
class App:
    def __init__(self, root, link, calib):
        self.root, self.link, self.calib = root, link, calib
        self.detector = Detector(link, calib)
        root.title("EDI PIPE CAM — Stereo Control Panel (rev 3)")
        root.configure(bg="#1e1e1e")

        # ---- video ----
        vid = tk.Frame(root, bg="#1e1e1e")
        vid.pack()
        self.lblL = tk.Label(vid, text="waiting for LEFT...", bg="black",
                             fg="#888", width=44, height=16)
        self.lblR = tk.Label(vid, text="waiting for RIGHT...", bg="black",
                             fg="#888", width=44, height=16)
        self.lblL.grid(row=0, column=0, padx=4, pady=4)
        self.lblR.grid(row=0, column=1, padx=4, pady=4)

        self.status = tk.Label(root, text="", bg="#1e1e1e", fg="#0f0")
        self.status.pack()

        # ---- telemetry ----
        tf = tk.LabelFrame(root, text="Telemetry (stepper odometry + IMU)",
                           bg="#1e1e1e", fg="#ccc")
        tf.pack(fill="x", padx=6, pady=4)
        self.lbl_odo = tk.Label(tf, text="dist: --- mm", bg="#1e1e1e",
                                fg="#fff", font=("Menlo", 12, "bold"))
        self.lbl_odo.grid(row=0, column=0, padx=8)
        self.lbl_imu = tk.Label(tf, text="pitch: ---  roll: ---",
                                bg="#1e1e1e", fg="#ccc")
        self.lbl_imu.grid(row=0, column=1, padx=8)
        self.lbl_state = tk.Label(tf, text="drv:-- act:-- hook:--",
                                  bg="#1e1e1e", fg="#ccc")
        self.lbl_state.grid(row=0, column=2, padx=8)
        self.lbl_slip = tk.Label(tf, text="", bg="#1e1e1e", fg="#e8a33d")
        self.lbl_slip.grid(row=0, column=3, padx=8)
        tk.Button(tf, text="ZERO ODOM",
                  command=lambda: self.link.send("Z")).grid(row=0, column=4, padx=8)

        # ---- drive + actuator (press & hold) ----
        drive = tk.Frame(root, bg="#1e1e1e")
        drive.pack(pady=4)
        self._hold_btn(drive, "▲ FORWARD", "F", "S", 0, 1)
        self._hold_btn(drive, "▼ BACKWARD", "B", "S", 2, 1)
        self._hold_btn(drive, "EXTEND", "E", "X", 1, 0)
        self._hold_btn(drive, "RETRACT", "R", "X", 1, 2)
        tk.Button(drive, text="STOP ALL", width=12, bg="#8b0000", fg="white",
                  command=self.stop_all).grid(row=1, column=1, padx=4, pady=2)

        # ---- servo (latching, single click) ----
        sf = tk.Frame(root, bg="#1e1e1e")
        sf.pack(pady=2)
        tk.Button(sf, text="HOOK", width=12,
                  command=lambda: self.link.send("H")).grid(row=0, column=0, padx=4)
        tk.Button(sf, text="RELEASE", width=12,
                  command=lambda: self.link.send("U")).grid(row=0, column=1, padx=4)
        tk.Button(sf, text="ASSEMBLY (90)", width=14,
                  command=lambda: self.link.send("A")).grid(row=0, column=2, padx=4)

        # ---- lining pull workflow ----
        pf = tk.LabelFrame(root, text="Lining pull (deploy -> mark -> back until green)",
                           bg="#1e1e1e", fg="#ccc")
        pf.pack(fill="x", padx=6, pady=4)
        tk.Button(pf, text="MARK PULL START",
                  command=self.mark_pull).grid(row=0, column=0, padx=6)
        tk.Label(pf, text="crack length (cm):", bg="#1e1e1e",
                 fg="#ccc").grid(row=0, column=1)
        self.ent_len = tk.Entry(pf, width=6)
        self.ent_len.insert(0, "10")
        self.ent_len.grid(row=0, column=2, padx=2)
        tk.Label(pf, text=f"+{PULL_MARGIN_CM:.0f} margin", bg="#1e1e1e",
                 fg="#ccc").grid(row=0, column=3)
        self.lbl_pull = tk.Label(pf, text="pulled: --- cm", bg="#1e1e1e",
                                 fg="#fff", font=("Menlo", 12, "bold"))
        self.lbl_pull.grid(row=0, column=4, padx=10)
        self.pull_start_mm = None

        # ---- capture ----
        cap = tk.Frame(root, bg="#1e1e1e")
        cap.pack(pady=4)
        self.rec_btn = tk.Button(cap, text="● REC", width=10, fg="#f55",
                                 command=self.toggle_record)
        self.rec_btn.grid(row=0, column=0, padx=4)
        tk.Button(cap, text="CALIB PAIR",
                  command=self.save_calib_pair).grid(row=0, column=1, padx=4)
        tk.Button(cap, text="SAVE (dataset)",
                  command=self.save_dataset).grid(row=0, column=2, padx=4)
        self.autocap = False
        self.btn_auto = tk.Button(cap, text="AUTO-CAP: OFF",
                                  command=self.toggle_autocap)
        self.btn_auto.grid(row=0, column=3, padx=4)

        # ---- live camera settings ----
        cf = tk.Frame(root, bg="#1e1e1e")
        cf.pack(pady=(0, 6))
        tk.Label(cf, text="cameras:", bg="#1e1e1e", fg="#ccc").grid(row=0, column=0)
        for i, (label, code) in enumerate(CAM_CMDS):
            tk.Button(cf, text=label, width=9,
                      command=lambda c=code: self.link.send(c)
                      ).grid(row=0, column=i + 1, padx=2)

        # ---- state ----
        self._held_cmd = None
        self.recording = False
        self.writers = [None, None]
        self.rec_paths = None
        self.rec_job = None
        self._last_autocap = 0
        for d in (REC_DIR, CALIB_DIR, DATASET_DIR):
            os.makedirs(d, exist_ok=True)

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._tick_video()
        self._tick_cmd()

    # ---- press-and-hold ----
    def _hold_btn(self, parent, text, cmd, stop_cmd, r, c):
        b = tk.Button(parent, text=text, width=12)
        b.grid(row=r, column=c, padx=4, pady=2)
        b.bind("<ButtonPress-1>", lambda e: self._start_hold(cmd))
        b.bind("<ButtonRelease-1>", lambda e: self._stop_hold(stop_cmd))

    def _start_hold(self, cmd):
        self._held_cmd = cmd
        self.link.send(cmd)

    def _stop_hold(self, stop_cmd):
        self._held_cmd = None
        self.link.send(stop_cmd)

    def _tick_cmd(self):
        if self._held_cmd:
            self.link.send(self._held_cmd)
        self.root.after(CMD_REPEAT_MS, self._tick_cmd)

    def stop_all(self):
        self._held_cmd = None
        self.link.send("S")
        self.link.send("X")

    # ---- lining pull ----
    def mark_pull(self):
        self.pull_start_mm = self.link.get_telem().get("mm")
        self.lbl_pull.config(text="pulled: 0.0 cm", fg="#fff")

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
            print(f"recording saved to {REC_DIR}")

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

    # ---- still capture ----
    def _save_pair(self, folder, tag):
        l, r, synced = self.link.get_pair()
        if l is None or r is None:
            print("pair not ready")
            return
        if not synced:
            print("WARNING: frames not synchronized — pair may be skewed")
        ts = int(time.time() * 1000)
        cv2.imwrite(os.path.join(folder, f"{tag}_{ts}_L.png"), l)
        cv2.imwrite(os.path.join(folder, f"{tag}_{ts}_R.png"), r)
        print(f"saved pair {ts} -> {folder}")

    def save_calib_pair(self):
        self._save_pair(CALIB_DIR, "calib")

    def save_dataset(self):
        self._save_pair(DATASET_DIR, "data")

    def toggle_autocap(self):
        self.autocap = not self.autocap
        self.btn_auto.config(text=f"AUTO-CAP: {'ON' if self.autocap else 'OFF'}")

    # ---- main loop ----
    def _tick_video(self):
        det = self.detector.get() if MODE != "none" else None
        if det:
            l, r, info = det
            msg = f"mode: {MODE}  |  {info}"
        else:
            l, r, _ = self.link.get_pair()
            msg = f"mode: {MODE}"
        if not self.calib.ok:
            msg += f"  |  {self.calib.error}"
        self.status.config(text=msg, fg="#0f0" if self.calib.ok else "#e8a33d")

        self._show(self.lblL, l)
        self._show(self.lblR, r)

        t = self.link.get_telem()
        if t:
            self.lbl_odo.config(text=f"dist: {t.get('mm', 0):.1f} mm")
            self.lbl_imu.config(
                text=f"pitch: {t.get('pitch', 0):+.1f}  roll: {t.get('roll', 0):+.1f}"
                     + ("" if t.get("imu") else "  (IMU MISSING)"))
            drv = {1: "FWD", -1: "BACK", 0: "stop"}.get(t.get("drv"), "?")
            act = {1: "EXT", -1: "RET", 0: "idle"}.get(t.get("act"), "?")
            hook = "HOOKED" if t.get("hook") else "RELEASED"
            self.lbl_state.config(text=f"drv:{drv}  act:{act}  {hook}")

            moving = t.get("drv", 0) != 0
            amag = t.get("amag", 1.0)
            self.lbl_slip.config(
                text="possible wheel slip" if moving and abs(amag - 1.0) < 0.01 else "")

            if self.pull_start_mm is not None and "mm" in t:
                pulled_cm = abs(t["mm"] - self.pull_start_mm) / 10.0
                try:
                    target = float(self.ent_len.get()) + PULL_MARGIN_CM
                except ValueError:
                    target = PULL_MARGIN_CM
                done = pulled_cm >= target
                self.lbl_pull.config(
                    text=f"pulled: {pulled_cm:.1f} / {target:.1f} cm"
                         + ("  COVERED" if done else ""),
                    fg="#0f0" if done else "#fff")

        if self.autocap and time.time() - self._last_autocap > AUTOCAP_SECS:
            self.save_dataset()
            self._last_autocap = time.time()

        self.root.after(50, self._tick_video)

    def _show(self, lbl, bgr):
        if bgr is None:
            return
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb)
        if SCALE != 1:
            img = img.resize((img.width * SCALE, img.height * SCALE),
                             Image.NEAREST)
        photo = ImageTk.PhotoImage(img)
        lbl.configure(image=photo, text="", width=img.width, height=img.height)
        lbl.image = photo          # keep a reference or Tk garbage-collects it

    def on_close(self):
        if self.recording:
            self.toggle_record()   # flush and close the video files
        self.stop_all()
        self.detector.running = False
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
        print("If it's busy, close the Arduino Serial Monitor / other panels.")
        return
    print("Using port:", port)

    link = FrameLink(ser)
    calib = StereoCalib(CALIB_FILE)
    if not calib.ok:
        print(calib.error + " — distances disabled.")

    root = tk.Tk()
    App(root, link, calib)
    root.mainloop()


if __name__ == "__main__":
    main()
