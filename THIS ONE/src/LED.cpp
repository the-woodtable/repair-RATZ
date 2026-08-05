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
    digitalWrite(_pin, HIGH);
    _state = true;
}

void LED::off() {
    digitalWrite(_pin, LOW);
    _state = false;
}

void LED::toggle() {
    if(_state){
        digitalWrite(_pin, LOW);
    }
    else{
        digitalWrite(_pin, HIGH);
    }
        
}

bool LED::isOn() const { return _state; }
int  LED::pin()  const { return _pin;   }