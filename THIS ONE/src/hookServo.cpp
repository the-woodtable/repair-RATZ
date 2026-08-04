// src/HookServo.cpp
#include "HookServo.h"

bool HookServo::begin(int pin) {
  pin_ = pin;

  servo_.setPeriodHertz(50);
  int channel = servo_.attach(pin_, 500, 2500);   // returns channel# or -1

  if (channel == -1) {
    return false;   // attach failed — pin invalid or no free PWM channel
  }

  setAngle(180);
  return true;
}
void HookServo::setAngle(int angle){
    servo_.write(angle);
    currentAngle_ = angle;
}

int HookServo::getCurrentAngle() {
    return currentAngle_;
}
