/*
  EDI PIPE CAM — dual-camera forwarder for the ESP32-S3 main board
  ----------------------------------------------------------------
  DROP-IN MODULE FOR THE INTEGRATOR — does not touch motor/actuator code.

  What it does:
    Reads the 0xAA55-framed JPEG streams from TWO ESP32-CAMs (one per
    UART), and forwards each complete frame over the native USB serial
    with a one-byte camera ID inserted after the magic:

      To PC:  0xAA 0x55 | ID ('L' or 'R') | uint32 LE length | JPEG

    Frames are forwarded atomically (never interleaved), so the PC can
    demux the two streams from the single USB port.

  How to integrate (3 lines in the existing sketch):

      #include "s3_cam_forwarder.h"

      void setup() {
        ...existing setup...
        camForwarderBegin();
      }

      void loop() {
        ...existing command handling / motor logic...
        camForwarderPump();     // call every loop iteration, non-blocking
      }

  Wiring (any two free RX-capable GPIOs — change the defines to match):
      LEFT  camera U0T ->  CAM_LEFT_RX_PIN   (default GPIO 18)
      RIGHT camera U0T ->  CAM_RIGHT_RX_PIN  (default GPIO 16)
      Both cameras: 5V + GND from the shared rail. TX to cams not needed.

  RAM use: 2 x 60 KB frame buffers from heap (S3 has 512 KB — fine).
*/

#pragma once
#include <Arduino.h>

#define CAM_LEFT_RX_PIN   18   // <-- integrator: set to your free pins
#define CAM_RIGHT_RX_PIN  16
#define CAM_BAUD      460800

namespace {

constexpr uint32_t CAM_MAX_FRAME = 60000;

class CamChannel {
 public:
  CamChannel(HardwareSerial &port, uint8_t id) : port_(port), id_(id) {}

  void begin(int rxPin) {
    port_.begin(CAM_BAUD, SERIAL_8N1, rxPin, -1);   // RX only
    port_.setRxBufferSize(4096);
    buf_ = (uint8_t *)malloc(CAM_MAX_FRAME);
  }

  // Non-blocking: consume whatever bytes are available, forward a frame
  // to USB serial once it is complete.
  void pump() {
    while (port_.available()) {
      uint8_t b = port_.read();
      switch (state_) {
        case SYNC1: if (b == 0xAA) state_ = SYNC2;                    break;
        case SYNC2: state_ = (b == 0x55) ? LEN : SYNC1; lenPos_ = 0;  break;
        case LEN:
          ((uint8_t *)&len_)[lenPos_++] = b;
          if (lenPos_ == 4) {
            if (len_ == 0 || len_ > CAM_MAX_FRAME || !buf_) { state_ = SYNC1; }
            else { pos_ = 0; state_ = BODY; }
          }
          break;
        case BODY:
          buf_[pos_++] = b;
          if (pos_ == len_) { emit(); state_ = SYNC1; }
          break;
      }
    }
  }

 private:
  void emit() {
    // Atomic tagged frame out the native USB CDC
    uint8_t hdr[7] = {0xAA, 0x55, id_};
    memcpy(hdr + 3, &len_, 4);
    Serial.write(hdr, 7);
    Serial.write(buf_, len_);
  }

  enum State { SYNC1, SYNC2, LEN, BODY };
  HardwareSerial &port_;
  uint8_t id_;
  State state_ = SYNC1;
  uint8_t *buf_ = nullptr;
  uint32_t len_ = 0, pos_ = 0;
  uint8_t lenPos_ = 0;
};

CamChannel camLeft(Serial1, 'L');
CamChannel camRight(Serial2, 'R');

}  // namespace

inline void camForwarderBegin() {
  camLeft.begin(CAM_LEFT_RX_PIN);
  camRight.begin(CAM_RIGHT_RX_PIN);
}

inline void camForwarderPump() {
  camLeft.pump();
  camRight.pump();
}
