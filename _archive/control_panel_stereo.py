"""
EDI PIPE CAM — PC Control Panel (Stereo Edition)
------------------------------------------------
    pip install pyserial pillow opencv-python numpy
    python control_panel_stereo.py

Same single serial port as before (the ESP32-S3). The S3 now forwards
BOTH camera streams, ID-tagged (see s3_cam_forwarder.h). Commands and
keepalive behaviour are unchanged from the original panel.

Distance:
    - Needs stereo_calib.npz in this folder (run stereo_calibrate.py once).
    - Without it: dual video only, no distance.
    - With it: views are rectified, disparity via SGBM, live distance
      shown at the crosshair. CLICK anywhere on the LEFT view to move
      the crosshair. Robot should be stationary when you trust a reading
      (the two streams are not frame-synced).
"""

import io
import os
import threading
import time
import tkinter as tk

import numpy as np
import cv2
import serial
from PIL import Image, ImageTk

from stereo_serial import TaggedFrameReader

# ------------------- Settings -------------------
PORT = None             # None = auto-detect; or set e.g. "/dev/cu.usbmodem101"
BAUD = 921600
KEEPALIVE_MS = 100
CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "stereo_calib.npz")
SCALE = 2               # 320x240 -> 640x480 display
REC_FPS = 10            # recording sample rate (frames duplicated if slower)
# Save recordings somewhere easy to find in Finder
REC_DIR = os.path.expanduser("~/Downloads/pipe_cam_recordings")


# ------------------- Stereo depth -------------------
class StereoDepth:
    """Loads calibration, rectifies pairs, computes disparity + distance."""

    def __init__(self, path):
        self.ok = False
        if not os.path.exists(path):
            return
        c = np.load(path)
        size = tuple(int(v) for v in c["image_size"])
        r1, r2, p1, p2, _, _, _ = cv2.stereoRectify(
            c["K1"], c["D1"], c["K2"], c["D2"], size, c["R"], c["T"],
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
        self.map_l = cv2.initUndistortRectifyMap(c["K1"], c["D1"], r1, p1,
                                                 size, cv2.CV_16SC2)
        self.map_r = cv2.initUndistortRectifyMap(c["K2"], c["D2"], r2, p2,
                                                 size, cv2.CV_16SC2)
        self.f = p1[0, 0]                          # rectified focal (px)
        self.baseline = float(np.linalg.norm(c["T"]))  # mm
        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=64, blockSize=7,
            P1=8 * 49, P2=32 * 49, uniquenessRatio=10,
            speckleWindowSize=100, speckleRange=2, disp12MaxDiff=1)
        self.ok = True

    def rectify(self, gray_l, gray_r):
        return (cv2.remap(gray_l, *self.map_l, cv2.INTER_LINEAR),
                cv2.remap(gray_r, *self.map_r, cv2.INTER_LINEAR))

    def disparity(self, rect_l, rect_r):
        return self.sgbm.compute(rect_l, rect_r).astype(np.float32) / 16.0

    def distance_mm(self, disp, x, y):
        """Median disparity in a 9x9 patch around (x, y) -> distance."""
        h, w = disp.shape
        x0, x1 = max(0, x - 4), min(w, x + 5)
        y0, y1 = max(0, y - 4), min(h, y + 5)
        patch = disp[y0:y1, x0:x1]
        valid = patch[patch > 0.5]
        if valid.size < 10:
            return None
        return (self.f * self.baseline) / float(np.median(valid))


class DepthWorker(threading.Thread):
    """Grabs latest L/R pair, rectifies, computes disparity ~5x/s.
    Results are read by the GUI thread."""

    def __init__(self, reader, depth):
        super().__init__(daemon=True)
        self.reader = reader
        self.depth = depth
        self.lock = threading.Lock()
        self.rect_l = self.rect_r = self.disp = None
        self.raw_l = self.raw_r = None
        self.running = True

    def run(self):
        last_l = last_r = None
        while self.running:
            jl, jr = self.reader.latest(b"L"), self.reader.latest(b"R")
            if jl is not None:
                last_l = jl
            if jr is not None:
                last_r = jr
            if last_l is None or last_r is None:
                time.sleep(0.02)
                continue

            cl = cv2.imdecode(np.frombuffer(last_l, np.uint8),
                              cv2.IMREAD_COLOR)
            cr = cv2.imdecode(np.frombuffer(last_r, np.uint8),
                              cv2.IMREAD_COLOR)
            if cl is None or cr is None:
                continue

            if self.depth.ok:
                gl = cv2.cvtColor(cl, cv2.COLOR_BGR2GRAY)
                gr = cv2.cvtColor(cr, cv2.COLOR_BGR2GRAY)
                rl, rr = self.depth.rectify(gl, gr)
                disp = self.depth.disparity(rl, rr)
                # Rectify the color views too so display matches disparity
                cl = cv2.remap(cl, *self.depth.map_l, cv2.INTER_LINEAR)
                cr = cv2.remap(cr, *self.depth.map_r, cv2.INTER_LINEAR)
                with self.lock:
                    self.rect_l, self.rect_r, self.disp = cl, cr, disp
            else:
                with self.lock:
                    self.raw_l, self.raw_r = cl, cr
            # Depth (SGBM) is the expensive part -> keep it ~5 Hz, but let
            # plain video update as fast as frames arrive.
            time.sleep(0.15 if self.depth.ok else 0.03)

    def snapshot(self):
        with self.lock:
            if self.depth.ok:
                return self.rect_l, self.rect_r, self.disp
            return self.raw_l, self.raw_r, None

    def stop(self):
        self.running = False


# ------------------- GUI -------------------
class ControlPanel:
    def __init__(self, root, ser):
        self.root = root
        self.ser = ser
        self.active_cmd = None
        self.keepalive_job = None
        self.photo_l = self.photo_r = None
        self.target = None   # crosshair (x, y) in image coords; None=center
        self.recording = False
        self.writers = [None, None]
        self.rec_paths = None
        self.rec_job = None

        root.title("EDI PIPE CAM — Stereo Control Panel")
        root.configure(bg="#1e1e1e")

        # Two video panes, left is clickable
        self.video_l = tk.Label(root, bg="black", text="waiting for LEFT...",
                                fg="#888", width=40, height=15)
        self.video_r = tk.Label(root, bg="black", text="waiting for RIGHT...",
                                fg="#888", width=40, height=15)
        self.video_l.grid(row=0, column=0, columnspan=2, padx=(10, 5), pady=10)
        self.video_r.grid(row=0, column=2, columnspan=2, padx=(5, 10), pady=10)
        self.video_l.bind("<Button-1>", self.on_click_left)

        self.dist_label = tk.Label(root, text="distance: --",
                                   fg="#ff0", bg="#1e1e1e",
                                   font=("TkDefaultFont", 14, "bold"))
        self.dist_label.grid(row=1, column=0, columnspan=4)

        self.status = tk.Label(root, text="idle", fg="#0f0", bg="#1e1e1e")
        self.status.grid(row=2, column=0, columnspan=4)

        buttons = [
            ("▲ FORWARD",  "F", "S"),
            ("▼ BACKWARD", "B", "S"),
            ("⇤ DEPLOY",   "D", "X"),
            ("⇥ RETRACT",  "R", "X"),
        ]
        for col, (label, press_cmd, release_cmd) in enumerate(buttons):
            btn = tk.Button(root, text=label, width=12, height=3,
                            bg="#333", fg="white", activebackground="#0a84ff")
            btn.grid(row=3, column=col, padx=6, pady=10)
            btn.bind("<ButtonPress-1>", lambda e, c=press_cmd: self.press(c))
            btn.bind("<ButtonRelease-1>", lambda e, c=release_cmd: self.release(c))

        stop_btn = tk.Button(root, text="STOP ALL", width=38, height=2,
                             bg="#8b0000", fg="white", command=self.stop_all)
        stop_btn.grid(row=4, column=0, columnspan=3, padx=(10, 4), pady=(0, 10))

        self.rec_btn = tk.Button(root, text="● REC", width=12, height=2,
                                 bg="#333", fg="#f55",
                                 command=self.toggle_record)
        self.rec_btn.grid(row=4, column=3, padx=(4, 10), pady=(0, 10))

        # Video + depth pipeline
        self.depth = StereoDepth(CALIB_FILE)
        if not self.depth.ok:
            self.dist_label.config(
                text="no stereo_calib.npz — video only (run stereo_calibrate.py)",
                fg="#f80")
        self.reader = TaggedFrameReader(ser)
        self.reader.start()
        self.worker = DepthWorker(self.reader, self.depth)
        self.worker.start()
        self.poll_video()

        root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---- command handling (unchanged behaviour) ----
    def send(self, cmd):
        try:
            self.ser.write(cmd.encode())
        except serial.SerialException:
            self.status.config(text="serial error", fg="#f00")

    def press(self, cmd):
        self.active_cmd = cmd
        self.send(cmd)
        self.status.config(text=f"holding: {cmd}", fg="#0af")
        self.schedule_keepalive()

    def release(self, stop_cmd):
        self.active_cmd = None
        if self.keepalive_job:
            self.root.after_cancel(self.keepalive_job)
            self.keepalive_job = None
        self.send(stop_cmd)
        self.status.config(text="idle", fg="#0f0")

    def schedule_keepalive(self):
        if self.active_cmd is not None:
            self.send(self.active_cmd)
            self.keepalive_job = self.root.after(KEEPALIVE_MS,
                                                 self.schedule_keepalive)

    def stop_all(self):
        self.active_cmd = None
        if self.keepalive_job:
            self.root.after_cancel(self.keepalive_job)
            self.keepalive_job = None
        self.send("S")
        self.send("X")
        self.status.config(text="STOPPED", fg="#f80")

    # ---- recording ----
    def toggle_record(self):
        if not self.recording:
            ts = time.strftime("%Y%m%d_%H%M%S")
            os.makedirs(REC_DIR, exist_ok=True)
            self.rec_paths = [os.path.join(REC_DIR, f"left_{ts}.mp4"),
                              os.path.join(REC_DIR, f"right_{ts}.mp4")]
            self.writers = [None, None]   # created lazily on first frame
            self.recording = True
            self.rec_btn.config(text="■ STOP REC", bg="#8b0000", fg="white")
            self.status.config(text="recording...", fg="#f55")
            self.rec_tick()
        else:
            self.recording = False
            if self.rec_job:
                self.root.after_cancel(self.rec_job)
                self.rec_job = None
            for w in self.writers:
                if w is not None:
                    w.release()
            self.writers = [None, None]
            self.rec_btn.config(text="● REC", bg="#333", fg="#f55")
            self.status.config(text=f"saved to {REC_DIR}/", fg="#0f0")

    def rec_tick(self):
        if not self.recording:
            return
        img_l, img_r, _ = self.worker.snapshot()
        for i, img in enumerate((img_l, img_r)):
            if img is None:
                continue
            if self.writers[i] is None:
                h, w = img.shape[:2]
                self.writers[i] = cv2.VideoWriter(
                    self.rec_paths[i], cv2.VideoWriter_fourcc(*"mp4v"),
                    REC_FPS, (w, h))
            self.writers[i].write(img)
        self.rec_job = self.root.after(int(1000 / REC_FPS), self.rec_tick)

    # ---- video / distance ----
    def on_click_left(self, event):
        self.target = (event.x // SCALE, event.y // SCALE)

    def poll_video(self):
        img_l, img_r, disp = self.worker.snapshot()

        if img_l is not None:
            h, w = img_l.shape[:2]
            tx, ty = self.target if self.target else (w // 2, h // 2)

            # Distance at crosshair
            if disp is not None:
                d = self.depth.distance_mm(disp, tx, ty)
                self.dist_label.config(
                    text=f"distance: {d / 10:.1f} cm" if d else
                         "distance: -- (no texture / too close)",
                    fg="#ff0" if d else "#f80")

            # Draw crosshair on left view
            vis = cv2.cvtColor(img_l, cv2.COLOR_BGR2RGB)
            cv2.drawMarker(vis, (tx, ty), (255, 255, 0),
                           cv2.MARKER_CROSS, 15, 1)
            self.photo_l = self._to_photo(vis)
            self.video_l.config(image=self.photo_l, text="",
                                width=w * SCALE, height=h * SCALE)

        if img_r is not None:
            h, w = img_r.shape[:2]
            self.photo_r = self._to_photo(
                cv2.cvtColor(img_r, cv2.COLOR_BGR2RGB))
            self.video_r.config(image=self.photo_r, text="",
                                width=w * SCALE, height=h * SCALE)

        self.root.after(30, self.poll_video)

    @staticmethod
    def _to_photo(rgb):
        img = Image.fromarray(rgb)
        img = img.resize((img.width * SCALE, img.height * SCALE),
                         Image.NEAREST)
        return ImageTk.PhotoImage(img)

    def on_close(self):
        if self.recording:
            self.toggle_record()   # flush + close video files
        self.stop_all()
        self.worker.stop()
        self.reader.stop()
        try:
            self.ser.close()
        except serial.SerialException:
            pass
        self.root.destroy()


def find_port():
    """Return PORT if set, else auto-detect a likely ESP32 serial port."""
    if PORT:
        return PORT
    from serial.tools import list_ports
    candidates = [p.device for p in list_ports.comports()
                  if any(k in p.device.lower()
                         for k in ("usbmodem", "usbserial", "ttyacm", "ttyusb"))]
    if not candidates:
        return None
    if len(candidates) > 1:
        print("Multiple serial devices found:")
        for c in candidates:
            print("   ", c)
        print(f"Using {candidates[0]} — set PORT at the top to override.")
    return candidates[0]


def main():
    port = find_port()
    if port is None:
        print("No ESP32 serial port found. Is the S3 plugged in via USB?")
        return
    try:
        ser = serial.Serial(port, BAUD, timeout=0.05)
    except serial.SerialException as e:
        print(f"Could not open {port}: {e}")
        print("If it's in use, close the Arduino Serial Monitor and retry.")
        return
    print(f"Connected to {port}")

    root = tk.Tk()
    ControlPanel(root, ser)
    root.mainloop()


if __name__ == "__main__":
    main()
