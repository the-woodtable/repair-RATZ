"""
control_panel_stereo2.py — EDI PIPE CAM camera panel
=====================================================
Capture and calibration only. No motor / servo / actuator controls —
robot driving lives in the teammate's panel.

What this does:
  * Live left/right video from the two ESP32-CAMs
  * REC          -> record both feeds to MP4 (footage review)
  * SAVE / AUTO-CAP -> synchronized PNG stills (YOLO training set)
  * CALIB PAIR   -> checkerboard pairs for stereo calibration
  * click the LEFT view -> distance to that point (needs stereo_calib.npz)

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

PORT   = None          # None = auto-detect, or "/dev/cu.usbmodem101"
BAUD   = 921600        # ignored by USB CDC but pyserial requires it
SCALE  = 2             # display zoom (320x240 -> 640x480)

CALIB_FILE  = os.path.join(HERE, "stereo_calib.npz")
REC_DIR     = os.path.expanduser("~/Downloads/pipe_cam_recordings")
DATASET_DIR = os.path.expanduser("~/Downloads/pipe_cam_dataset")
CALIB_DIR   = os.path.expanduser("~/Downloads/pipe_cam_calib_pairs")

REC_FPS      = 10
AUTOCAP_SECS = 1.0
PAIR_MAX_AGE = 0.30    # L/R further apart than this = unsynced


# ============================ SERIAL ============================
def find_port():
    if PORT:
        return PORT
    cands = [p.device for p in serial.tools.list_ports.comports()
             if any(k in p.device.lower()
                    for k in ("usbmodem", "usbserial", "ttyacm", "ttyusb"))]
    if not cands:
        return None
    if len(cands) > 1:
        print("Multiple serial devices:", ", ".join(cands))
        print(f"Using {cands[0]} — set PORT at the top to override.")
    return cands[0]


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
        self.dropped = 0                # frames rejected as corrupt
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
                continue

            # Telemetry must be consumed, not skipped, or its bytes get
            # mistaken for image data. (Not displayed in this panel.)
            if raw_id == self.TELEM_ID:
                body = self._read_exact(length)
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
                continue

            payload = head2 + self._read_exact(length - 2)
            if len(payload) != length:
                continue

            # Sensor pads bytes after FFD9; an end marker far too early means
            # the frame is spliced from pieces of different frames.
            end = payload.rfind(b"\xFF\xD9")
            if end < 0 or end + 2 < length * 0.5:
                self.dropped += 1
                continue

            try:
                self._jpegq.put_nowait((cam, payload[:end + 2], time.time()))
            except queue.Full:
                pass

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
                    self.counts[cam] += 1

    def get_pair(self):
        with self.lock:
            l, r = self.latest[0], self.latest[1]
        if l is None or r is None:
            return (l[0] if l else None), (r[0] if r else None), False
        return l[0], r[0], abs(l[1] - r[1]) <= PAIR_MAX_AGE

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
            self.baseline = float(d["baseline"])       # mm
            self.sgbm = cv2.StereoSGBM_create(
                minDisparity=0, numDisparities=64, blockSize=7,
                P1=8 * 49, P2=32 * 49, uniquenessRatio=10,
                speckleWindowSize=100, speckleRange=2, disp12MaxDiff=1)
            self.ok = True
        except KeyError:
            self.error = ("stereo_calib.npz is an old format — "
                          "re-run stereo_calibrate.py")

    def rectify(self, left, right):
        if not self.ok:
            return left, right
        return (cv2.remap(left, self.map1x, self.map1y, cv2.INTER_LINEAR),
                cv2.remap(right, self.map2x, self.map2y, cv2.INTER_LINEAR))

    def disparity(self, rect_l, rect_r):
        gl = cv2.cvtColor(rect_l, cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(rect_r, cv2.COLOR_BGR2GRAY)
        return self.sgbm.compute(gl, gr).astype(np.float32) / 16.0

    def distance_mm(self, disp, x, y):
        h, w = disp.shape
        x0, x1 = max(0, x - 4), min(w, x + 5)
        y0, y1 = max(0, y - 4), min(h, y + 5)
        patch = disp[y0:y1, x0:x1]
        good = patch[patch > 0.5]
        if good.size < 10:
            return None
        return self.fx * self.baseline / float(np.median(good))


class DepthWorker(threading.Thread):
    """Rectify + disparity at ~5 Hz (SGBM is the expensive part)."""

    def __init__(self, link, calib):
        super().__init__(daemon=True)
        self.link, self.calib = link, calib
        self.lock = threading.Lock()
        self.view_l = self.view_r = self.disp = None
        self.running = True

    def run(self):
        while self.running:
            l, r, _ = self.link.get_pair()
            if l is None and r is None:
                time.sleep(0.05)
                continue
            # Depth needs BOTH cameras. With only one connected, still show
            # its video — recording and dataset capture work fine mono.
            if self.calib.ok and l is not None and r is not None:
                rl, rr = self.calib.rectify(l, r)
                d = self.calib.disparity(rl, rr)
                with self.lock:
                    self.view_l, self.view_r, self.disp = rl, rr, d
                time.sleep(0.15)
            else:
                with self.lock:
                    self.view_l, self.view_r, self.disp = l, r, None
                time.sleep(0.03)

    def snapshot(self):
        with self.lock:
            return self.view_l, self.view_r, self.disp


# ============================ GUI ============================
class App:
    def __init__(self, root, link, calib):
        self.root, self.link, self.calib = root, link, calib
        self.worker = DepthWorker(link, calib)
        self.worker.start()

        root.title("EDI PIPE CAM — camera panel (capture + calibration)")
        root.configure(bg="#1e1e1e")

        # ---- video ----
        vid = tk.Frame(root, bg="#1e1e1e")
        vid.pack(padx=8, pady=8)
        self.lblL = tk.Label(vid, text="waiting for LEFT...", bg="black",
                             fg="#888", width=44, height=16)
        self.lblR = tk.Label(vid, text="waiting for RIGHT...", bg="black",
                             fg="#888", width=44, height=16)
        self.lblL.grid(row=0, column=0, padx=4)
        self.lblR.grid(row=0, column=1, padx=4)
        self.lblL.bind("<Button-1>", self.on_click_left)

        self.lbl_dist = tk.Label(root, text="distance: --", bg="#1e1e1e",
                                 fg="#ff0", font=("Menlo", 15, "bold"))
        self.lbl_dist.pack()
        self.lbl_status = tk.Label(root, text="", bg="#1e1e1e", fg="#0f0")
        self.lbl_status.pack()

        # ---- capture ----
        cap = tk.LabelFrame(root, text="Capture", bg="#1e1e1e", fg="#ccc")
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
        self.target = None
        self.recording = False
        self.writers = [None, None]
        self.rec_paths = None
        self.rec_job = None
        self._last_autocap = 0
        self._fps_t0 = time.time()
        self._fps_base = (0, 0)
        for d in (REC_DIR, DATASET_DIR, CALIB_DIR):
            os.makedirs(d, exist_ok=True)

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._tick()

    # ---- distance ----
    def on_click_left(self, event):
        self.target = (event.x // SCALE, event.y // SCALE)

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

    # ---- main loop ----
    def _tick(self):
        img_l, img_r, disp = self.worker.snapshot()

        if img_l is not None:
            h, w = img_l.shape[:2]
            tx, ty = self.target if self.target else (w // 2, h // 2)
            if disp is not None:
                d = self.calib.distance_mm(disp, tx, ty)
                self.lbl_dist.config(
                    text=f"distance: {d / 10:.1f} cm" if d
                         else "distance: -- (no texture / too close)",
                    fg="#ff0" if d else "#e8a33d")
            vis = img_l.copy()
            cv2.drawMarker(vis, (tx, ty), (0, 255, 255), cv2.MARKER_CROSS, 15, 1)
            self._show(self.lblL, vis)
        if img_r is not None:
            self._show(self.lblR, img_r)

        # fps + health line
        now = time.time()
        if now - self._fps_t0 >= 1.0:
            cl, cr = self.link.counts[0], self.link.counts[1]
            fl = (cl - self._fps_base[0]) / (now - self._fps_t0)
            fr = (cr - self._fps_base[1]) / (now - self._fps_t0)
            self._fps_base = (cl, cr)
            self._fps_t0 = now
            msg = f"L {fl:.1f} fps   R {fr:.1f} fps   dropped {self.link.dropped}"
            if self.recording:
                msg += "   ● RECORDING"
            if not self.calib.ok:
                msg += f"   |  {self.calib.error}"
            self.lbl_status.config(
                text=msg, fg="#0f0" if self.calib.ok else "#e8a33d")

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
    calib = StereoCalib(CALIB_FILE)
    if not calib.ok:
        print(calib.error + " — distance readout disabled.")

    root = tk.Tk()
    App(root, link, calib)
    root.mainloop()


if __name__ == "__main__":
    main()
