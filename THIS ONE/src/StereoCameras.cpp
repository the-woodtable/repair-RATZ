// src/StereoCameras.cpp
#include "StereoCameras.h"

static const uint32_t CAM_MAX_FRAME = 60000;

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
          state = (len > 0 && len < CAM_MAX_FRAME && buf) ? 3 : 0;
        }
        break;
      case 3:  // collecting JPEG body
        buf[got++] = b;
        if (got == len) {
          emit();
          state = 0;
          return;          // fair share — one frame per call
        }
        break;
    }
  }
}

void StereoCameras::Channel::emit() {
  uint8_t h[7] = { 0xAA, 0x55, id,
                   (uint8_t)len, (uint8_t)(len >> 8),
                   (uint8_t)(len >> 16), (uint8_t)(len >> 24) };
  Serial.write(h, 7);
  Serial.write(buf, len);
  frames++;
}

// ---------------- StereoCameras ----------------

bool StereoCameras::begin(int leftRx, int leftTx, int rightRx, int rightTx) {
  bool okL = left_.begin(&Serial1, leftRx, leftTx, 0);    // id 0 = LEFT
  bool okR = right_.begin(&Serial2, rightRx, rightTx, 1); // id 1 = RIGHT
  return okL && okR;
}

void StereoCameras::update() {
  left_.pump();
  right_.pump();
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
  Serial.write(h, 7);
  Serial.write((const uint8_t*)json, n);
}
