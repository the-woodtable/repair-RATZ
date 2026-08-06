// src/StereoCameras.cpp
#include "StereoCameras.h"

// ===================== FAULT-ISOLATION TOGGLES =====================
// For chasing a fault that stays on ONE side after the camera module, the
// ribbon and the modules themselves have all been swapped. Flip ONE at a
// time, reflash the S3 only, and compare diagnostic reports.
//
//   SWAP_UARTS       same pins, opposite UART peripheral
//                      fault moves -> UART peripheral / driver
//                      fault stays -> the pin, wire, or camera power
//   PUMP_RIGHT_FIRST reverses servicing order
//                      fault moves -> CPU starvation, not the channel
//                      fault stays -> genuinely channel-specific
//
// Leave BOTH at 0 for normal operation.
#define SWAP_UARTS       0
#define PUMP_RIGHT_FIRST 0
// ===================================================================

static const uint32_t CAM_MAX_FRAME = 60000;
static const uint32_t CAM_MIN_FRAME = 512;   // smaller than any real frame

// ---------------- Channel ----------------

bool StereoCameras::Channel::begin(HardwareSerial* s, int rxPin, int txPin,
                                   uint8_t camId) {
  ser = s;
  id = camId;
  // Big RX buffer: bytes pile up here while the CPU is busy writing the
  // other camera's frame to USB. Too small -> silent drops -> torn frames.
  // MUST be called BEFORE begin() or it has no effect.
  ser->setRxBufferSize(32768);
  ser->begin(StereoCameras::CAM_BAUD, SERIAL_8N1, rxPin, txPin);
  buf = (uint8_t*)malloc(CAM_MAX_FRAME);
  return buf != nullptr;
}

void StereoCameras::Channel::pump() {
  // Camera sends: AA 55 | len(4 LE) | JPEG   (no id — identity comes from
  // which UART this is). Forward AT MOST ONE frame, then return, so the
  // other camera and the main loop get their turn.
  while (ser->available()) {
    uint8_t b = ser->read();
    switch (state) {
      case 0:  // waiting for 0xAA
        state = (b == 0xAA) ? 1 : 0;
        break;
      case 1:  // waiting for 0x55
        if (b == 0x55) { state = 2; hdrIdx = 0; }
        else state = (b == 0xAA) ? 1 : 0;
        break;
      case 2:  // collecting 4 length bytes
        hdr[hdrIdx++] = b;
        if (hdrIdx == 4) {
          len = (uint32_t)hdr[0] | ((uint32_t)hdr[1] << 8) |
                ((uint32_t)hdr[2] << 16) | ((uint32_t)hdr[3] << 24);
          got = 0;
          // Sanity range as well as the buffer limit: a real QVGA JPEG is
          // never a few hundred bytes. A corrupted length that still looks
          // plausible would otherwise swallow the frames that follow.
          state = (len >= CAM_MIN_FRAME && len < CAM_MAX_FRAME && buf) ? 3 : 0;
        }
        break;
      case 3:  // collecting JPEG body
        // Every JPEG starts FF D8. Checking the first two bytes turns a
        // corrupted header into a 2-byte loss instead of eating the next
        // ~60 KB (which showed up as the stream freezing after one glitch).
        if (got == 0 && b != 0xFF) { state = 0; break; }
        if (got == 1 && b != 0xD8) { state = 0; break; }
        buf[got++] = b;

        // BULK READ the rest of the body. Reading a 4 KB frame one byte at a
        // time costs ~4000 driver calls; with two cameras that is ~70k calls
        // per second, which starves the loop and overflows the UART buffers
        // (bytes lost mid-frame -> grey-bottom images). readBytes() pulls
        // whatever is already buffered in one go.
        if (got >= 2 && got < len) {
          int avail = ser->available();
          if (avail > 0) {
            uint32_t want = len - got;
            if ((uint32_t)avail < want) want = avail;
            got += ser->readBytes(buf + got, want);
          }
        }

        if (got == len) {
          emit();
          state = 0;
          return;          // fair share — one frame per call
        }
        break;
    }
  }
}

// Serial.write() on the native USB CDC can return SHORT when the host isn't
// draining fast enough — it does not guarantee it wrote everything. Ignoring
// that meant we announced "N bytes follow" and then sent fewer, so the PC
// parser read past the end of the frame, swallowed the next frame's header,
// and desynced. That showed up as most of the traffic being unparseable.
// Always loop until the whole buffer is out.
// Two hazards to avoid at once:
//   1. Serial.write() on USB CDC can return SHORT. Ignoring that meant we
//      announced "N bytes follow" and sent fewer, desyncing the PC parser.
//   2. Waiting for the host to catch up BLOCKS the loop, and while we're
//      blocked nobody drains the camera UARTs — their buffers overflow and
//      the next frames arrive with bytes missing (grey-bottom images).
// So: only start a frame if the whole thing already fits in the TX buffer.
// If it doesn't, skip this frame entirely. A dropped frame is free; a
// partial frame corrupts the stream, and blocking corrupts the other camera.
static bool writeWhole(const uint8_t* hdr, size_t hlen,
                       const uint8_t* body, size_t blen) {
  if ((size_t)Serial.availableForWrite() < hlen + blen) return false;
  size_t sent = 0;
  while (sent < hlen) {                       // fits, so these complete
    size_t n = Serial.write(hdr + sent, hlen - sent);
    if (n == 0) return false;
    sent += n;
  }
  sent = 0;
  while (sent < blen) {
    size_t n = Serial.write(body + sent, blen - sent);
    if (n == 0) return false;
    sent += n;
  }
  return true;
}

void StereoCameras::Channel::emit() {
  uint8_t h[7] = { 0xAA, 0x55, id,
                   (uint8_t)len, (uint8_t)(len >> 8),
                   (uint8_t)(len >> 16), (uint8_t)(len >> 24) };
  if (writeWhole(h, 7, buf, len)) frames++;
  else                            skipped++;   // host too slow; frame dropped
}

// ---------------- StereoCameras ----------------

bool StereoCameras::begin(int leftRx, int leftTx, int rightRx, int rightTx) {
  // The USB TX buffer must hold an ENTIRE frame, because writeWhole() only
  // sends a frame when it fits (never partially). Default is a few hundred
  // bytes — far too small, which would skip every frame.
  Serial.setTxBufferSize(TX_BUFFER_BYTES);

#if SWAP_UARTS
  // Same PINS, opposite UART peripherals. If a fault that is stuck on one
  // side moves when you flip this, the cause is the UART peripheral or its
  // driver; if it stays put, the cause is the pin or what's wired to it.
  bool okL = left_.begin(&Serial2, leftRx, leftTx, 0);
  bool okR = right_.begin(&Serial1, rightRx, rightTx, 1);
#else
  bool okL = left_.begin(&Serial1, leftRx, leftTx, 0);    // id 0 = LEFT
  bool okR = right_.begin(&Serial2, rightRx, rightTx, 1); // id 1 = RIGHT
#endif
  return okL && okR;
}

void StereoCameras::update() {
  // Servicing order matters under load: whichever channel is pumped second
  // has had longer to accumulate bytes, so it overflows first if the CPU is
  // the bottleneck. Flip PUMP_RIGHT_FIRST to see whether the fault follows
  // the ORDER (a starvation/CPU problem) rather than the channel.
#if PUMP_RIGHT_FIRST
  right_.pump();
  left_.pump();
#else
  left_.pump();
  right_.pump();
#endif
}

bool StereoCameras::handleChar(char c) {
  // Camera commands arrive as 3 bytes: '@' + target + cmd.
  // target: 'l' left, 'r' right, 'a' all.  cmd: see camera_firmware.ino
  // (q/Q quality, b/B brightness, c/C contrast, 1-4 resolution, f/F LED).
  // Escaped with '@' because bare letters collide with drive commands.
  switch (cmdState_) {
    case 0:
      if (c == '@') { cmdState_ = 1; return true; }
      return false;                      // not ours — let main.cpp handle it
    case 1:
      cmdTarget_ = c;
      cmdState_ = 2;
      return true;
    case 2:
      if (cmdTarget_ == 'l' || cmdTarget_ == 'a') left_.ser->write(c);
      if (cmdTarget_ == 'r' || cmdTarget_ == 'a') right_.ser->write(c);
      cmdState_ = 0;
      return true;
  }
  return false;
}

void StereoCameras::sendTelemetryFrame(const char* json) {
  uint32_t n = strlen(json);
  uint8_t h[7] = { 0xAA, 0x55, 2,
                   (uint8_t)n, (uint8_t)(n >> 8),
                   (uint8_t)(n >> 16), (uint8_t)(n >> 24) };
  writeWhole(h, 7, (const uint8_t*)json, n);   // skipped if it won't fit
}
