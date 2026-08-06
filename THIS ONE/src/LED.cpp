#include "LED.h"

// 20 kHz, well above anything the cameras can see.
//
// THIS MATTERS FOR THE CAMERAS. A PWM'd lamp is strobing, and a rolling
// shutter exposes each row at a slightly different time. If the PWM period
// is comparable to the exposure time, different rows catch different numbers
// of pulses and you get horizontal brightness banding in every frame — which
// would poison the YOLO training set and wreck stereo matching. At 20 kHz
// the period is 50 us, so even a short exposure integrates many whole
// cycles and every row sees the same average light.
//
// 20 kHz is also above human hearing, so no coil whine from the wiring.
static const int  LED_PWM_HZ  = 20000;
static const int  LED_PWM_BITS = 8;     // 0-255 duty
static const int  LED_DUTY_MAX = 255;

bool LED::begin(int lightpin, int channel) {
    if (lightpin < 0) return false;

    _pin = lightpin;
    _ch  = channel;

    // Arduino-ESP32 core 2.x API. (Core 3.x replaced this with
    // ledcAttach(pin, freq, bits); this project is on 2.x — the same
    // version issue that bit actuator.cpp.)
    ledcSetup(_ch, LED_PWM_HZ, LED_PWM_BITS);
    ledcAttachPin(_pin, _ch);

    _state  = false;
    _bright = 255;
    ledcWrite(_ch, 0);         // start off, known state
    return true;
}

void LED::applyDuty_() {
    if (_pin < 0) return;
    if (!_state || _bright == 0) {
        ledcWrite(_ch, 0);
        return;
    }
    // Gamma ~2.0. Perceived brightness follows roughly the square root of
    // power, so squaring the request cancels that out and a slider at 50%
    // actually looks like half brightness.
    uint32_t duty = ((uint32_t)_bright * _bright) / 255;
    if (duty == 0) duty = 1;               // never round a lit LED to off
    if (duty > LED_DUTY_MAX) duty = LED_DUTY_MAX;
    ledcWrite(_ch, duty);
}

void LED::on() {
    _state = true;
    if (_bright == 0) _bright = 255;       // "on" must actually light up
    applyDuty_();
}

void LED::off() {
    _state = false;
    applyDuty_();
}

void LED::toggle() {
    _state = !_state;
    applyDuty_();
}

void LED::setBrightness(uint8_t b) {
    _bright = b;
    // Setting a level implies you want it lit; setting 0 implies off.
    _state = (b > 0);
    applyDuty_();
}

uint8_t LED::brightness() const { return _bright; }
bool    LED::isOn()      const { return _state;  }
int     LED::pin()       const { return _pin;    }
