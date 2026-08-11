#pragma once
#include <ESP32Servo.h>

class HookServo {
public:

  bool begin(int pin, int initial_angle);
  void setAngle(int angle);
  int getCurrentAngle();

  // Which LEDC channel ESP32Servo handed this servo. Two servos MUST get
  // different channels, and neither may match the LED (4) or actuator (6).
  // Printed at boot so a clash is visible instead of being guessed at.
  int getChannel() const { return channel_; }

private:
  int pin_;
  int currentAngle_;
  int channel_ = -1;
  Servo servo_;
};