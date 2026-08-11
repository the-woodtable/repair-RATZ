#include "Actuator.h"

// ===========================================================================
// PROJECT-WIDE LEDC CHANNEL MAP — keep this in sync or things cross-trigger.
//
// LEDC channels are PAIRED onto timers: 0&1 -> timer0, 2&3 -> timer1,
// 4&5 -> timer2, 6&7 -> timer3. Channels sharing a timer share a FREQUENCY,
// so claiming one can silently break its partner.
//
//   timer0  ch 0,1  ESP32Servo  (hookServo, spoolServo) - allocated dynamically
//   timer1  ch 2,3  ESP32Servo  (spare)
//   timer2  ch 4,5  LED         (see LED.cpp)
//   timer3  ch 6,7  Actuator    (here)
//
// THIS WAS CHANNEL 0 AND IT WAS A BUG. ESP32Servo allocates channels from 0
// upward and has no idea about raw ledcSetup() calls, so it had already given
// channel 0 to hookServo. This ledcSetup(0,...) then reconfigured that same
// channel to 20 kHz AND attached the actuator's PWM pin to it — so moving a
// servo drove the actuator, and driving the actuator moved the servo.
// Reconfiguring timer0 also broke spoolServo on channel 1.
// main.cpp now calls ESP32PWM::allocateTimer(0/1) to pin the servos down.
static const int ACT_PWM_CH = 6;   // LEDC channel used for the actuator


bool Actuator::begin(int ain1, int ain2, int pwm, int stby){
    ain1_ = ain1;
    ain2_ = ain2;
    pwm_ = pwm;
    stby_ = stby;

    pinMode(ain1_, OUTPUT);
    pinMode(ain2_, OUTPUT);
    pinMode(stby_, OUTPUT);

    ledcSetup(ACT_PWM_CH, 20000, 8);   // use the constant, not a literal
    ledcAttachPin(pwm_, ACT_PWM_CH);

    // BUG WAS HERE: ain1_ was written twice and ain2_ never at all, so one
    // H-bridge direction input was left in an undefined state at boot. With
    // AIN1/AIN2 indeterminate the TB6612 can drive on noise alone — which is
    // why the actuator appeared to move when unrelated keys were pressed.
    digitalWrite(ain1_, LOW);
    digitalWrite(ain2_, LOW);          // <- was a duplicate of ain1_
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
  lastCommandMs_ = millis();
}

void Actuator::retract(int speed){
  if (speed < 0)   speed = 0;
  if (speed > 255) speed = 255;

  digitalWrite(ain1_, LOW);
  digitalWrite(ain2_, HIGH);
  ledcWrite(ACT_PWM_CH, speed);

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