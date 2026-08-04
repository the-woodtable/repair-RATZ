#include "Actuator.h"

bool begin(int ain1, int ain2, int pwm, int stby){
    ain1_ = ain1;
    ain2_ = ain2;
    pwm_ = pwm;

}
void Actuator::extend(int speed) {

  lastCommandMs_ = millis();

  if (speed < 0)   speed = 0;
  if (speed > 255) speed = 255;

  digitalWrite(ain1_, HIGH);
  digitalWrite(ain2_, LOW);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pwm_, speed);
#else
  ledcWrite(PWM_CHANNEL, speed);
#endif

  state_ = EXTENDING;
  currentSpeed_ = speed;
}

void retract(int speed);

void stop();
int getState();