"""
robot_console.py — text-only console for the S3 (no video)
===========================================================
WHY THIS EXISTS
    `pio device monitor` is unusable once the cameras are powered, because
    the SAME usb port carries the JPEG video stream. The monitor faithfully
    prints those bytes and you get a screen of garbage symbols. Turning the
    bench supply off "fixes" it only because it kills the cameras.

    This script reads the same port but UNDERSTANDS the frame protocol:
        0xAA 0x55 | id(1) | len(4 LE) | payload
    so it can throw the image frames away and show you just the telemetry
    text — while still letting you type commands.

USE
    source ~/Downloads/esp32env/bin/activate
    cd ~/Desktop/30.007/"camera codes"
    python3 robot_console.py

    Then type a command letter and press Enter:
        H hook       U release    A assembly
        F forward    B back       S stop drive
        D deploy     R retract    X stop actuator
        L led on     O led off    Z zero odometry
        q quit  (sends S and X first)

    Several at once is fine: type  FS  to pulse the drive.

NOTE: only ONE program may hold the port. Close the panel (and any serial
monitor) before running this, and close this before running the panel.
"""

import sys
import threading
import time

import serial
import serial.tools.list_ports

PORT = None        # None = auto-detect
BAUD = 921600      # ignored by USB CDC, but pyserial wants a number

MAGIC = b"\xAA\x55"
MAX_FRAME = 300_000

VALID = set("HUAFBSDRXLOZ")


def find_port():
    """Prefer the S3's native USB CDC over any USB-serial adapter.

    /dev/cu.usbserial* is the CH340/FTDI you flash the CAMERAS through —
    grabbing it here makes esptool report "port is busy".
    """
    if PORT:
        return PORT
    ports = [p.device for p in serial.tools.list_ports.comports()]
    native = [d for d in ports
              if any(k in d.lower() for k in ("usbmodem", "ttyacm"))]
    adapter = [d for d in ports
               if any(k in d.lower() for k in ("usbserial", "ttyusb"))]
    if not native and not adapter:
        return None
    if not native:
        print(f"No usbmodem port; falling back to {adapter[0]}. "
              "Close this before flashing a camera.")
        return adapter[0]
    if len(native) > 1 or adapter:
        print("Ports seen:", ", ".join(ports), "— using", native[0])
    return native[0]


class Reader(threading.Thread):
    """Strips camera frames, prints everything else as text."""

    daemon = True

    def __init__(self, ser):
        super().__init__()
        self.ser = ser
        self.running = True
        self.frames = 0
        self.frame_bytes = 0

    def _read_exact(self, n):
        buf = bytearray()
        while self.running and len(buf) < n:
            chunk = self.ser.read(n - len(buf))
            if chunk:
                buf.extend(chunk)
            else:
                break          # timed out — give up rather than hang
        return bytes(buf)

    def run(self):
        sync = bytearray()
        text = bytearray()
        while self.running:
            try:
                b = self.ser.read(1)
            except serial.SerialException as e:
                print(f"\n[port died: {e}]")
                return
            if not b:
                continue

            sync += b
            if len(sync) > 2:
                # The byte falling out of the sync window was not part of a
                # header, so it is real text — keep it.
                text.append(sync.pop(0))

            if bytes(sync) == MAGIC:
                sync.clear()
                self._flush(text)
                head = self._read_exact(5)
                if len(head) < 5:
                    continue
                length = int.from_bytes(head[1:5], "little")
                if 0 < length <= MAX_FRAME:
                    self._read_exact(length)      # discard the image
                    self.frames += 1
                    self.frame_bytes += length
                continue

            if b in (b"\n", b"\r"):
                # The last 1-2 bytes are still parked in the sync window.
                # A newline can't be part of the magic, so it's safe to
                # release them — otherwise the line's final character
                # (usually "}") gets stranded onto the next line.
                text.extend(sync)
                sync.clear()
                self._flush(text)

    def _flush(self, text):
        if not text:
            return
        line = bytes(text).decode("ascii", "replace").strip()
        text.clear()
        if line:
            print(line)


def main():
    port = find_port()
    if port is None:
        print("No ESP32 serial port found. Is the S3 plugged in?")
        return
    try:
        ser = serial.Serial(port, BAUD, timeout=0.05)
    except serial.SerialException as e:
        print(f"Could not open {port}: {e}")
        print("Something else is holding it — close the panel or the "
              "serial monitor first.")
        return

    print(f"Connected to {port}. Type command letters + Enter. 'q' to quit.")
    print("  H U A  servo   |  F B S  drive  |  D R X  actuator")
    print("  L O    led     |  Z      zero   |  ?      frame counter\n")

    rx = Reader(ser)
    rx.start()

    try:
        for raw in sys.stdin:
            s = raw.strip()
            if s == "q":
                break
            if s == "?":
                print(f"[camera frames discarded: {rx.frames}, "
                      f"{rx.frame_bytes/1e6:.1f} MB]")
                continue
            for ch in s:
                up = ch.upper()
                if up in VALID:
                    ser.write(up.encode())
                    print(f"[sent {up}]")
                    time.sleep(0.01)
                elif not ch.isspace():
                    print(f"[ignored '{ch}' — not a command]")
    except KeyboardInterrupt:
        pass

    # main.cpp has no dead-man watchdog: motors keep running until told to
    # stop, so never exit without stopping them.
    print("\nstopping motors...")
    ser.write(b"S")
    ser.write(b"X")
    time.sleep(0.2)
    rx.running = False
    ser.close()


if __name__ == "__main__":
    main()
