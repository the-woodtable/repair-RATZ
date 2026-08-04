#include "Actuator.h"

static const int ACT_PWM_CH = 0;   // LEDC channel used for the actuator


bool Actuator::begin(int ain1, int ain2, int pwm, int stby){
    ain1_ = ain1;
    ain2_ = ain2;
    pwm_ = pwm;
    stby_ = stby;

    pinMode(ain1_, OUTPUT);
    pinMode(ain2_, OUTPUT);
    pinMode(stby_, OUTPUT);

    // Arduino-ESP32 core 2.x API: setup the channel, then bind the pin.
    // ledcSetup returns the actual frequency, or 0 if it failed.
    if (ledcSetup(ACT_PWM_CH, 20000, 8) == 0) {   // 20 kHz, 8-bit (0-255)
        return false;                             // no free channel
    }
    ledcAttachPin(pwm_, ACT_PWM_CH);
 
    digitalWrite(stby_, HIGH);            // wake the TB6612 out of standby
 
    stop();
    lastCommandMs_ = millis();
  
    return true;


}
void Actuator::extend(int speed) {


  if (speed < 0)   speed = 0;
  if (speed > 255) speed = 255;

  digitalWrite(ain1_, HIGH);
  digitalWrite(ain2_, LOW);
  ledcWrite(ACT_PWM_CH, speed);

  state_ = 1;
}

void Actuator::retract(int speed){
  if (speed < 0)   speed = 0;
  if (speed > 255) speed = 255;

  digitalWrite(ain1_, LOW);
  digitalWrite(ain2_, HIGH);
  ledcWrite(ACT_PWM_CH, speed);

  state_ = -1;
}

void Actuator::stop() {
  digitalWrite(ain1_, LOW);
  digitalWrite(ain2_, LOW);
  state_ = 0;
}

int Actuator::getState(){
  return state_;
}