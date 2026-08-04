#include <Arduino.h>
#include "HookServo.h"

int PIN_SERVO = 11;

bool setupSuccessful = true;

HookServo hookServo;

void setup() {
  Serial.begin(921600);

  if (!hookServo.begin(PIN_SERVO)) {
    Serial.println("ERROR: hook servo failed to attach!");
    setupSuccessful = false;
  }

}

void loop() {
  // hookServo.setAngle(180);
  // delay(1000);
  // hookServo.setAngle(0);
  // delay(1000);
  while (Serial.available()) {
    char c = Serial.read();

    switch (c) {
      case 'H':
        hookServo.setAngle(180);
        break;
      case 'U':
        hookServo.setAngle(0);
        break;
      case 'A':
        hookServo.setAngle(90);
        break;
      default:
        break;   // ignore unrecognized characters
    }
  }
}

