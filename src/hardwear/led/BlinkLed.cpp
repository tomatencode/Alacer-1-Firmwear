#include <Arduino.h>
#include "BlinkLed.hpp"

BlinkLed::BlinkLed(int pin, uint8_t brightness, uint32_t defaultOnDuration_ms)
    : _pin(pin), _defaultOnDuration_ms(defaultOnDuration_ms), _flashDuration_ms(0), _brightness(brightness) {
}

void BlinkLed::begin() {
    pinMode(_pin, OUTPUT);
    analogWrite(_pin, 0);
}

void BlinkLed::update() {
    if (millis() - _flashtime >= _flashDuration_ms) {
        analogWrite(_pin, 0);
    }
}

void BlinkLed::flash() {
    flash(_defaultOnDuration_ms);
}

void BlinkLed::flash(uint32_t duration_ms) {
    analogWrite(_pin, _brightness);
    _flashtime = millis();
    _flashDuration_ms = duration_ms;
}