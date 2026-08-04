#include "LED.h"

bool LED::begin(int lightpin) {
    if (lightpin < 0) return false;

    _pin = lightpin;
    pinMode(_pin, OUTPUT);

    _state = false;
    digitalWrite(_pin, LOW);   // start off, known state
    return true;
}

void LED::on() {
    set(true);
}

void LED::off() {
    set(false);
}

void LED::toggle() {
    set(!_state);
}

void LED::set(bool state) {
    if (_pin < 0) return;      // begin() was never called
    _state = state;
    digitalWrite(_pin, _state ? HIGH : LOW);
}

bool LED::isOn() const { return _state; }
int  LED::pin()  const { return _pin;   }