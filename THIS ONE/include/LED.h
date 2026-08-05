#pragma once
#include <Arduino.h>

class LED {
public:
    bool begin(int lightpin);

    void on();
    void off();
    void toggle();

    bool isOn() const;
    int  pin() const;

private:
    int  _pin   = -1;      // -1 = not initialised yet
    bool _state = false;
};   