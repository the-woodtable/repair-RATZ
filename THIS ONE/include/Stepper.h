#pragma once
#include <Arduino.h>

class Stepper {
public:
  // stepPin/dirPin/enPin: wiring to the DRV8825
  // stepsPerRev: 200 for a standard NEMA 17 in full-step mode
  // stepHz: how many full steps per second at full speed (drive speed)
  bool begin(int stepPin, int dirPin, int enPin, int stepHz);

  void setDrive(int8_t dir);   // -1 = backward, 0 = stop, +1 = forward
  void zero();                 // reset step count to 0

  // Change speed while running. Safe to call mid-move: it only rewrites the
  // timer alarm period, so the step count and direction are untouched.
  //
  // CAUTION: the odometer counts pulses SENT, not steps actually taken. Wind
  // this up past what the motor can pull and it silently misses steps, so the
  // distance reading drifts without anything looking wrong. Test the top of
  // the range in the pipe, loaded, before relying on it.
  void setStepHz(int hz);
  int  getStepHz() const;

  long getStepCount() const;
  int8_t getDrive() const;

private:
  int pin_step_;
  int pin_dir_;
  int pin_en_;
  int stepsPerRev_;
  int stepHz_ = 0;         // last commanded speed, so the panel can read it back

  hw_timer_t* timer_ = nullptr;

  // volatile: written inside the ISR, read outside it — without this the
  // compiler could cache a stale value and never see the ISR's updates.
  volatile int8_t  dir_       = 0;      // -1, 0, +1 — mirrors DIR pin state
  volatile bool    stepLevel_ = false;  // current STEP pin level, toggled each tick
  volatile long    stepCount_ = 0;      // signed odometer, +forward

  // ISR trampoline machinery 
  static Stepper* instance_;
  static void IRAM_ATTR isrTrampoline();
  void IRAM_ATTR handleTimer();
};