"""
EDI PIPE CAM — PC Control Panel (Stepper Edition)
--------------------------------------------------
Run in VS Code / terminal:
    pip install pyserial pillow
    python control_panel.py

Set PORT below to your ESP32-S3's serial port:
    Windows: "COM5" etc. (Device Manager -> Ports)
    macOS:   "/dev/cu.usbmodemXXXX"
    Linux:   "/dev/ttyACM0"

Controls (all press-and-hold; releasing stops that motion):
    FORWARD  -> 'F'  (M1 CW, M2 CCW)      release -> 'S'
    BACKWARD -> 'B'  (M1 CCW, M2 CW)      release -> 'S'
    DEPLOY   -> 'D'  (actuator extends)   release -> 'X'
    RETRACT  -> 'R'  (actuator retracts)  release -> 'X'
While a button is held, the command is re-sent every 100 ms as a
keepalive — the firmware stops everything if the stream goes silent.

Video: parses 0xAA55-framed JPEG stream forwarded by the S3.
"""

import io
import struct
import threading
import queue
import tkinter as tk

import serial
from PIL import Image, ImageTk

# ------------------- Settings -------------------
PORT = "COM11"          # <-- CHANGE ME
BAUD = 921600          # ignored by native USB CDC, but must be set
KEEPALIVE_MS = 100     # command repeat interval while a button is held
MAX_FRAME = 200_000    # sanity limit on frame length (bytes)
MAGIC = b"\xAA\x55"


# ------------------- Serial frame reader -------------------
class FrameReader(threading.Thread):
    """Background thread: scans the serial stream for 0xAA55-framed JPEGs
    and keeps only the newest complete frame in a 1-slot queue."""

    def __init__(self, ser: serial.Serial, frame_q: "queue.Queue[bytes]"):
        super().__init__(daemon=True)
        self.ser = ser
        self.frame_q = frame_q
        self.running = True

    def _read_exact(self, n: int) -> bytes:
        buf = bytearray()
        while self.running and len(buf) < n:
            chunk = self.ser.read(n - len(buf))
            if chunk:
                buf.extend(chunk)
        return bytes(buf)

    def run(self):
        sync = bytearray()
        while self.running:
            b = self.ser.read(1)
            if not b:
                continue
            sync += b
            if len(sync) > 2:
                del sync[0]
            if bytes(sync) != MAGIC:
                continue
            sync.clear()

            header = self._read_exact(4)
            if len(header) < 4:
                continue
            (length,) = struct.unpack("<I", header)
            if not (0 < length <= MAX_FRAME):
                continue  # garbage length -> resync on next magic

            payload = self._read_exact(length)
            if len(payload) != length:
                continue

            # Keep only the freshest frame
            try:
                self.frame_q.get_nowait()
            except queue.Empty:
                pass
            self.frame_q.put(payload)

    def stop(self):
        self.running = False


# ------------------- GUI -------------------
class ControlPanel:
    def __init__(self, root: tk.Tk, ser: serial.Serial):
        self.root = root
        self.ser = ser
        self.active_cmd = None       # command char currently held
        self.keepalive_job = None
        self.photo = None            # keep reference so Tk doesn't GC it

        root.title("EDI PIPE CAM — Control Panel")
        root.configure(bg="#1e1e1e")

        # Video area
        self.video_label = tk.Label(root, bg="black",
                                    text="waiting for video...",
                                    fg="#888", width=48, height=18)
        self.video_label.grid(row=0, column=0, columnspan=4,
                              padx=10, pady=10, sticky="nsew")

        self.status = tk.Label(root, text="idle", fg="#0f0", bg="#1e1e1e")
        self.status.grid(row=1, column=0, columnspan=4)

        # Press-and-hold buttons: (label, press-cmd, release-cmd)
        buttons = [
            ("▲ FORWARD",  "F", "S"),
            ("▼ BACKWARD", "B", "S"),
            ("⇤ DEPLOY",   "D", "X"),
            ("⇥ RETRACT",  "R", "X"),
        ]
        for col, (label, press_cmd, release_cmd) in enumerate(buttons):
            btn = tk.Button(root, text=label, width=12, height=3,
                            bg="#333", fg="white",
                            activebackground="#0a84ff")
            btn.grid(row=2, column=col, padx=6, pady=10)
            btn.bind("<ButtonPress-1>",
                     lambda e, c=press_cmd: self.press(c))
            btn.bind("<ButtonRelease-1>",
                     lambda e, c=release_cmd: self.release(c))

        # Emergency stop (single click stops everything)
        stop_btn = tk.Button(root, text="STOP ALL", width=54, height=2,
                             bg="#8b0000", fg="white",
                             command=self.stop_all)
        stop_btn.grid(row=3, column=0, columnspan=4, padx=10, pady=(0, 10))

        # Video pipeline
        self.frame_q: "queue.Queue[bytes]" = queue.Queue(maxsize=1)
        self.reader = FrameReader(ser, self.frame_q)
        self.reader.start()
        self.poll_video()

        root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---- command handling ----
    def send(self, cmd: str):
        try:
            self.ser.write(cmd.encode())
        except serial.SerialException:
            self.status.config(text="serial error", fg="#f00")

    def press(self, cmd: str):
        self.active_cmd = cmd
        self.send(cmd)
        self.status.config(text=f"holding: {cmd}", fg="#0af")
        self.schedule_keepalive()

    def release(self, stop_cmd: str):
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

    # ---- video ----
    def poll_video(self):
        try:
            jpeg = self.frame_q.get_nowait()
        except queue.Empty:
            jpeg = None

        if jpeg:
            try:
                img = Image.open(io.BytesIO(jpeg))
                img = img.resize((img.width * 2, img.height * 2),
                                 Image.NEAREST)  # 320x240 -> 640x480
                self.photo = ImageTk.PhotoImage(img)
                self.video_label.config(image=self.photo, text="",
                                        width=img.width, height=img.height)
            except Exception:
                pass  # corrupt frame; skip it

        self.root.after(30, self.poll_video)

    def on_close(self):
        self.stop_all()
        self.reader.stop()
        try:
            self.ser.close()
        except serial.SerialException:
            pass
        self.root.destroy()


def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.05)
    except serial.SerialException as e:
        print(f"Could not open {PORT}: {e}")
        print("Edit PORT at the top of this file to match your ESP32-S3.")
        return

    root = tk.Tk()
    ControlPanel(root, ser)
    root.mainloop()


if __name__ == "__main__":
    main()