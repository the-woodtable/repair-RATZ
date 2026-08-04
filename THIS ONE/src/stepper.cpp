#include "Stepper.h"

// Static member must be defined outside the class, exactly once, in the .cpp.
// Starts null until begin() sets it.
Stepper* Stepper::instance_ = nullptr;

bool Stepper::begin(int stepPin, int dirPin, int enPin, int stepHz) {
  pin_step_    = stepPin;
  pin_dir_     = dirPin;
  pin_en_      = enPin;
  stepsPerRev_ = 200;

  pinMode(pin_step_, OUTPUT);
  pinMode(pin_dir_,  OUTPUT);
  pinMode(pin_en_,   OUTPUT);

  digitalWrite(pin_en_, LOW);   // DRV8825 EN is active-LOW: LOW = driver on, holding torque

  instance_ = this;   // so the static trampoline knows which object to call into


 // core 2.x API: timerBegin(timer_number, prescaler, count_up)
  timer_ = timerBegin(0, 80, true);
  if (timer_ == nullptr) {
    return false;
  }

  timerAttachInterrupt(timer_, &Stepper::isrTrampoline, true);

  uint64_t intervalUs = 1000000UL / (stepHz * 2);
  timerAlarmWrite(timer_, intervalUs, true);
  timerAlarmEnable(timer_);

  return true;

}


void Stepper::setDrive(int8_t dir) {
  if (dir == -1) {
    digitalWrite(pin_dir_, LOW); 
  }
  else if (dir == 1){
    digitalWrite(pin_dir_, HIGH);  
  }

  dir_ = dir;
}

void Stepper::zero() {
  noInterrupts();       // pause interrupts briefly so the ISR can't fire mid-write
  stepCount_ = 0;
  interrupts();
}

long Stepper::getStepCount() const {
  // Reading a `long` isn't guaranteed atomic on all platforms, so we still
  // guard it — cheap insurance against a torn read if the ISR fires mid-copy.
  noInterrupts();
  long count = stepCount_;
  interrupts();
  return count;
}

int8_t Stepper::getDrive() const {
  return dir_;
}

// --- ISR machinery ---

void IRAM_ATTR Stepper::isrTrampoline() {
  if (instance_ != nullptr) {
    instance_->handleTimer();
  }
}

void IRAM_ATTR Stepper::handleTimer() {
  if (dir_ == 0) {
    // Not driving — make sure the pin rests LOW rather than left mid-pulse.
    if (stepLevel_) {
      digitalWrite(pin_step_, LOW);
      stepLevel_ = false;
    }
    return;
  }

  stepLevel_ = !stepLevel_;
  digitalWrite(pin_step_, stepLevel_);

  if (stepLevel_) {          // only count on the LOW->HIGH transition
    stepCount_ += dir_;
  }
}