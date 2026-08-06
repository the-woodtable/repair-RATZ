#pragma once
#include <Arduino.h>

// Inspection lamp on a PWM (LEDC) channel, so brightness is adjustable.
//
// on()/off()/toggle()/isOn()/pin() behave exactly as before, so existing
// callers keep working. on() restores the last brightness you set rather
// than always going to full — set it once, then toggle freely.
class LED {
public:
    // channel: LEDC channel. MUST NOT collide with anything else in the
    // project. Currently taken: 0 = actuator (see actuator.cpp), and
    // ESP32Servo grabs channels dynamically for the hook servo. 4 sits on
    // its own timer (channels pair 0/1, 2/3, 4/5, 6/7 -> timers 0..3), so
    // changing LED frequency can't disturb the actuator.
    bool begin(int lightpin, int channel = 4);

    void on();
    void off();
    void toggle();

    // 0 = off, 255 = full. Applies immediately AND becomes the level that
    // on() restores. Gamma-corrected so the steps look evenly spaced to
    // the eye — a raw linear duty cycle looks like it jumps to full
    // brightness in the first quarter of its range and then barely changes.
    void setBrightness(uint8_t b);
    uint8_t brightness() const;   // 0-255, the requested level

    bool isOn() const;
    int  pin() const;

private:
    void applyDuty_();

    int     _pin   = -1;      // -1 = not initialised yet
    int     _ch    = 4;
    bool    _state = false;
    uint8_t _bright = 255;    // last requested level; default full
};
