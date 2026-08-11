// src/HookServo.cpp
#include "HookServo.h"

bool HookServo::begin(int pin, int initial_angle) {
  pin_ = pin;

  servo_.setPeriodHertz(50);

  // 500-2500 us: the original range, restored. It was briefly narrowed to
  // 1000-2000 while chasing a dead hook servo, but the actual fault was a
  // loose connection. Wider range = fuller travel, so keep it unless a
  // POSITIONAL servo audibly strains at 0 or 180 (that would mean it is
  // being driven past its mechanical stop, and 1000-2000 is the safe retreat).
  channel_ = servo_.attach(pin_, 500, 2500);

  // ESP32Servo signals failure by returning 0, NOT -1. The old check
  // (channel == -1) therefore never fired, so a servo that failed to attach
  // reported success and then silently did nothing — no error, no movement.
  if (channel_ <= 0) {
    Serial.printf("HookServo: attach FAILED on GPIO %d (returned %d) — "
                  "no free PWM channel, or the pin is not output-capable\n",
                  pin_, channel_);
    return false;
  }

  Serial.printf("HookServo: GPIO %d attached, LEDC channel %d, 50 Hz\n",
                pin_, channel_);
  setAngle(initial_angle);
  return true;
}
void HookServo::setAngle(int angle){
    servo_.write(angle);
    currentAngle_ = angle;
}

int HookServo::getCurrentAngle() {
    return currentAngle_;
}
