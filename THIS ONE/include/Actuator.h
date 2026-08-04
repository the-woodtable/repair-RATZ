#pragma once
#include <Arduino.h>


class Actuator {
public:
  bool begin(int ain1, int ain2, int pwm, int stby);
  void extend(int speed);
  void retract(int speed);
  void stop();
  int getState();

  void update();

private:

  static const uint32_t MAX_RUN_MS = 10000;   // 10 second safety cutoff
  int ain1_;
  int ain2_;
  int pwm_;
  int stby_;
  int state_;
  uint32_t lastCommandMs_ = 0;
};

