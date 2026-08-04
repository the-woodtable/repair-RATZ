#pragma once
#include <Arduino.h>

/*
  StereoCameras — forwards both ESP32-CAM streams to the PC over USB.

  Matches the team's module pattern (HookServo / Stepper / Actuator):
  begin() returns false on failure, update() is called every loop,
  handleChar() consumes this module's serial commands.

  What it does:
    - Reads framed JPEGs from two ESP32-CAMs (UART1 = left, UART2 = right)
    - Re-emits each complete frame over USB Serial, tagged with a camera id,
      atomically (frames never interleave):
          0xAA 0x55 | id (0 = LEFT, 1 = RIGHT) | len (uint32 LE) | JPEG
    - Relays "@" camera-setting commands to the cameras (quality,
      brightness, resolution, flash LED — see camera_firmware.ino)

  Integration into main.cpp:

      #include "StereoCameras.h"
      StereoCameras cameras;

      // in setup():
      if (!cameras.begin(CAML_RX, CAML_TX, CAMR_RX, CAMR_TX)) { ... }

      // FIRST thing in the Serial.available() loop:
      char c = Serial.read();
      if (cameras.handleChar(c)) continue;   // it was a camera command
      switch (c) { ... existing cases ... }

      // every loop iteration:
      cameras.update();

  IMPORTANT — telemetry on a shared port: the PC panel parses BINARY frames
  from this same USB port. Plain Serial.println() telemetry works only if
  it never lands inside a frame — always print BETWEEN update() calls
  (i.e. from loop(), never from an ISR). For telemetry the panel can parse,
  wrap the JSON with sendTelemetryFrame() instead of println.

  Baud: CAM_BAUD below must equal Serial.begin() in camera_firmware.ino.
*/

class StereoCameras {
public:
  static const uint32_t CAM_BAUD = 921600;   // MUST equal Serial.begin() in
                                             // camera_firmware.ino. Change one,
                                             // change the other, reflash BOTH.

  // Returns false if a frame buffer could not be allocated.
  // TX pins are optional (pass -1): they only carry live-settings commands.
  bool begin(int leftRx, int leftTx, int rightRx, int rightTx);

  // Call every loop(). Forwards at most one frame per camera per call so
  // neither camera can starve the other (or the rest of the loop).
  void update();

  // Feed every received PC byte here first. Returns true if the byte was
  // part of a camera command ("@" + target + cmd) and is consumed.
  bool handleChar(char c);

  // Wrap a JSON string as a binary telemetry frame (id 2) the panel parses.
  void sendTelemetryFrame(const char* json);

  // For telemetry / debugging.
  uint32_t framesLeft()  const { return left_.frames; }
  uint32_t framesRight() const { return right_.frames; }

private:
  struct Channel {
    HardwareSerial* ser = nullptr;
    uint8_t  id = 0;
    uint8_t  state = 0, hdrIdx = 0, hdr[4];
    uint32_t len = 0, got = 0;
    uint8_t* buf = nullptr;
    uint32_t frames = 0;

    bool begin(HardwareSerial* s, int rxPin, int txPin, uint8_t camId);
    void pump();
    void emit();
  };

  Channel left_, right_;
  uint8_t cmdState_ = 0;   // 0 idle, 1 want target, 2 want cmd
  char    cmdTarget_ = 0;
};
