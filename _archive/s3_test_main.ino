/*
  EDI PIPE CAM — S3 bench-test sketch (cameras only, no motors)
  -------------------------------------------------------------
  Use this to verify both camera streams BEFORE integration.
  Put s3_cam_forwarder.h in the same sketch folder.

  Wiring for the bench test:
    LEFT  cam U0T -> GPIO 18   (or edit defines in s3_cam_forwarder.h)
    RIGHT cam U0T -> GPIO 16
    Both cams: 5V + GND (use a supply that can give ~1.5 A on 5V)

  Then on the laptop run control_panel_stereo.py — you should see both
  feeds. The motion buttons will send commands into the void; that's
  fine, nothing is listening.

  When your teammate integrates for real, they DON'T use this file —
  they add the same 3 lines (include / begin / pump) to their own sketch.
*/

#include "s3_cam_forwarder.h"

// NOTE: in Arduino IDE, Tools -> "USB CDC On Boot" MUST be "Enabled",
// otherwise Serial goes to UART pins instead of USB and the PC sees nothing.

void setup() {
  Serial.begin(460800);   // native USB CDC; baud value is ignored
  camForwarderBegin();
}

void loop() {
  camForwarderPump();

  // Heartbeat: proves this sketch is alive even if no cameras talk.
  // Harmless to the frame protocol (parser skips non-magic bytes).
  static uint32_t lastBeat = 0;
  if (millis() - lastBeat > 2000) {
    lastBeat = millis();
    Serial.print("HB\n");
  }
}
