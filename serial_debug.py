"""
serial_debug.py — what is actually arriving from the S3?
--------------------------------------------------------
Run with the panel CLOSED (only one program can hold the port):

    python3 serial_debug.py
    python3 serial_debug.py /dev/cu.usbserial-1140         # specific port
    python3 serial_debug.py /dev/cu.usbserial-1140 115200  # boot-message mode

Listens 10 s and reports every layer: heartbeats, telemetry, camera frames,
and whether those frames would pass the panel's JPEG check.

BOOT-MESSAGE MODE: pass 115200 as the second argument. Every ESP32 prints a
boot banner at 115200 on reset, whatever the sketch later sets. If you see
readable text there, the chip is alive — which separates "board is dead"
from "board is running but not streaming".
"""

import struct
import sys
import time

import serial
from serial.tools import list_ports

PORT = sys.argv[1] if len(sys.argv) > 1 else None
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 921600

# id byte -> stream name (robot/camera-only firmware uses 0/1/2; the very
# old bench sketch used b'L'/b'R')
CAM_IDS = {b"\x00": "LEFT", b"\x01": "RIGHT", b"L": "LEFT", b"R": "RIGHT"}
TELEM_ID = b"\x02"


def find_port():
    if PORT:
        return PORT
    cands = [p.device for p in list_ports.comports()
             if any(k in p.device.lower()
                    for k in ("usbmodem", "usbserial", "ttyacm", "ttyusb"))]
    for p in list_ports.comports():
        print(f"  seen: {p.device}  ({p.description})")
    return cands[0] if cands else None


def main():
    port = find_port()
    if not port:
        print("No serial port found — check the USB cable (must be a data cable).")
        return
    print(f"Listening on {port} at {BAUD} baud for 10 s...")
    if BAUD == 115200:
        print("(boot-message mode — power-cycle or reset the board NOW)\n")
    else:
        print()
    ser = serial.Serial(port, BAUD, timeout=0.1)
    data = bytearray()
    t0 = time.time()
    while time.time() - t0 < 10:
        data.extend(ser.read(4096))
    ser.close()
    data = bytes(data)

    print(f"total bytes: {len(data)}  ({len(data) / 10:.0f} B/s)")

    # Boot-message mode: just show whatever text arrived.
    if BAUD == 115200:
        text = data.decode("ascii", "replace")
        printable = sum(32 <= b < 127 or b in (10, 13) for b in data)
        print("--- raw text ---")
        print(text if text else "(nothing)")
        print("--- end ---")
        if data and printable > len(data) * 0.6:
            print("-> Readable boot text: THE CHIP IS ALIVE. It's a firmware "
                  "or sensor problem, not a dead board.")
        elif data:
            print("-> Bytes but not readable text: try power-cycling during "
                  "the 10 s window; otherwise suspect the board.")
        else:
            print("-> Nothing even at 115200: board is not running "
                  "(check 5V/GND at the board, then suspect damage).")
        return

    if not data:
        print("-> NOTHING arriving. S3 side: wrong port, USB CDC On Boot "
              "disabled, or sketch not flashed/running.")
        return

    heartbeats = data.count(b"HB\n")
    frames = {"LEFT": 0, "RIGHT": 0}
    valid = {"LEFT": 0, "RIGHT": 0}
    telem = 0
    bad_example = None
    unknown_magic = 0

    i = 0
    while True:
        i = data.find(b"\xAA\x55", i)
        if i < 0 or i + 7 > len(data):
            break
        raw_id = data[i + 2:i + 3]
        (length,) = struct.unpack("<I", data[i + 3:i + 7])
        if raw_id == TELEM_ID and 0 < length < 1000:
            telem += 1
            i += 7 + length
            continue
        name = CAM_IDS.get(raw_id)
        if name and 0 < length <= 300_000:
            frames[name] += 1
            payload = data[i + 7:i + 7 + length]
            if len(payload) == length:
                if payload[:2] == b"\xFF\xD8" and payload.rfind(b"\xFF\xD9") > 0:
                    valid[name] += 1
                elif bad_example is None:
                    bad_example = (payload[:4].hex(), payload[-8:].hex())
            i += 7 + length
            continue
        unknown_magic += 1
        i += 2

    print(f"heartbeats (HB): {heartbeats}   (expect ~5 if S3 alive; robot "
          "firmware has no heartbeat — telemetry plays that role)")
    print(f"telemetry frames: {telem}   (expect ~50 with robot firmware, "
          "0 with cameras-only sketch)")
    print(f"camera frames   L: {frames['LEFT']}   R: {frames['RIGHT']}")
    print(f"panel-valid     L: {valid['LEFT']}   R: {valid['RIGHT']}"
          "   <- what the panel will display")
    if bad_example:
        print(f"invalid frame example: starts {bad_example[0]} "
              f"ends ...{bad_example[1]}")
    if unknown_magic:
        print(f"stray magic hits: {unknown_magic}  (a few are normal — JPEG "
              "data contains AA55 by chance; hundreds = corruption)")

    print()
    if frames["LEFT"] or frames["RIGHT"]:
        silent = [n for n in ("LEFT", "RIGHT") if not frames[n]]
        if silent:
            print(f"-> {' and '.join(silent)} camera silent: its 5V/GND, "
                  "U0T wire, or it's boot-looping (watch its big white LED).")
        elif not (valid["LEFT"] or valid["RIGHT"]):
            print("-> frames arrive but ALL corrupt: baud mismatch — "
                  "camera_firmware Serial.begin() vs the S3's CAM_BAUD.")
        else:
            print("-> streams OK. If the panel still shows nothing, it may "
                  "be fighting another program for the port.")
    elif heartbeats or telem:
        print("-> S3 alive but NO camera data: camera 5V/GND, U0T->RX wires, "
              "common ground, or baud mismatch.")
        print("   Camera LED meanings:  big white LED blinking steadily "
              "1x/sec = camera module failed init (reseat the ribbon cable).")
        print("   Small red LED flickering/restarting = brownout — power "
              "supply can't hold 5V under load.")
    else:
        print("-> bytes arriving but nothing recognizable: wrong device on "
              "this port, or a very wrong baud somewhere.")


if __name__ == "__main__":
    main()
