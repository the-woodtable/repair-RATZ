#pragma once
#include <ESP32Servo.h>

class HookServo {
public:

  bool begin(int pin);  
  void setAngle(int angle);
  int getCurrentAngle();


private:
  int pin_;
  int currentAngle_;
  Servo servo_;
};