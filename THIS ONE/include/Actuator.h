#pragma once
#include <Arduino.h>


class Actuator {
public:
  bool begin(int ain1, int ain2, int pwm, int stby);
  void extend(int speed);
  void retract(int speed);
  void stop();
  int getState();

private:
  int ain1_;
  int ain2_;
  int pwm_;
  int stby_;
  int state_;
};

