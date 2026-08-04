#include "Actuator.h"

bool Actuator::begin(int ain1, int ain2, int pwm, int stby){
    ain1_ = ain1;
    ain2_ = ain2;
    pwm_ = pwm;
    stby_ = stby;

    pinMode(ain1_, OUTPUT);
    pinMode(ain2_, OUTPUT);
    pinMode(stby_, OUTPUT);

    ledcSetup(0, 20000, 8);        // pick a channel number unused elsewhere in the project
    ledcAttachPin(pwm_, 0);

    digitalWrite(ain1_, LOW);   
    digitalWrite(ain1_, LOW);   
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
  ledcWrite(pwm_, speed);

  state_ = 1;
  lastCommandMs_ = millis();
}

void Actuator::retract(int speed){
  if (speed < 0)   speed = 0;
  if (speed > 255) speed = 255;

  digitalWrite(ain1_, LOW);
  digitalWrite(ain2_, HIGH);
  ledcWrite(pwm_, speed);

  state_ = -1;
  lastCommandMs_ = millis();
}

void Actuator::stop() {
  digitalWrite(ain1_, LOW);
  digitalWrite(ain2_, LOW);
  state_ = 0;
}

void Actuator::update() {
  if (state_ != 0 && (millis() - lastCommandMs_ >= MAX_RUN_MS)) {
    stop();
  }
}

int Actuator::getState(){
  return state_;
}