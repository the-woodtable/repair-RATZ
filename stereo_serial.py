"""
Shared reader for the ID-tagged stereo stream from the ESP32-S3.

Wire format:
    0xAA 0x55 | ID byte | uint32 LE length | payload

Two firmwares are supported, so the same panel works on the bench and on
the finished robot:
    bench sketch (s3_cam_forwarder.h): id b'L' / b'R'
    robot firmware (pipe_robot_firmware.ino): id 0 = left, 1 = right,
                                              2 = telemetry JSON (skipped)

Used by stereo_calibrate.py. The main panel has its own richer reader
(FrameLink) with diagnostics; this one is the minimal version.
"""

import struct
import threading
import queue

MAGIC = b"\xAA\x55"
MAX_FRAME = 200_000

LEFT, RIGHT = b"L", b"R"
IDS = (LEFT, RIGHT)

# Everything the parser accepts -> which camera it means.
ID_MAP = {b"L": LEFT, b"R": RIGHT,        # bench sketch
          b"\x00": LEFT, b"\x01": RIGHT}  # robot firmware
TELEMETRY_ID = b"\x02"


class TaggedFrameReader(threading.Thread):
    """Background thread: demuxes tagged JPEG frames into per-camera
    1-slot queues (always holding only the newest frame)."""

    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser
        self.queues = {i: queue.Queue(maxsize=1) for i in IDS}
        self.telemetry = None   # newest telemetry JSON string, or None
        self.running = True

    def _read_exact(self, n):
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

            head = self._read_exact(5)          # ID + length
            if len(head) < 5:
                continue
            raw_id = head[:1]
            (length,) = struct.unpack("<I", head[1:5])
            if not (0 < length <= MAX_FRAME):
                continue                        # garbage -> resync

            # Telemetry frames must be consumed, not skipped, or their bytes
            # get mistaken for frame data.
            if raw_id == TELEMETRY_ID:
                body = self._read_exact(length)
                self.telemetry = body.decode("ascii", "replace")
                continue

            cam_id = ID_MAP.get(raw_id)
            if cam_id is None:
                continue                        # unknown id -> resync

            payload = self._read_exact(length)
            if len(payload) != length:
                continue

            # Must start with the JPEG start marker; the sensor pads garbage
            # after the image, so find the end marker anywhere and trim.
            if payload[:2] != b"\xFF\xD8":
                continue
            end = payload.rfind(b"\xFF\xD9")
            if end < 0:
                continue
            payload = payload[:end + 2]

            q = self.queues[cam_id]
            try:
                q.get_nowait()
            except queue.Empty:
                pass
            q.put(payload)

    def latest(self, cam_id):
        """Newest frame for b'L'/b'R', or None."""
        try:
            return self.queues[cam_id].get_nowait()
        except queue.Empty:
            return None

    def stop(self):
        self.running = False
