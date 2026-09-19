#include "Buzzer.hpp"

namespace hardware {

    Buzzer::Buzzer(uint8_t pin)
    : _pin(pin), _active(false),
      _frequency(0), _endTime(0),
      _melodyIndex(0) {}

void Buzzer::begin() {
    pinMode(_pin, OUTPUT);
    off();
}

void Buzzer::on(uint16_t frequency) {
    _frequency = frequency;
    tone(_pin, _frequency);
    _active = true;
    _endTime = 0;
    _melody.clear();
}

void Buzzer::off() {
    noTone(_pin);
    _active = false;
    _endTime = 0;
    _melody.clear();
}

void Buzzer::beep(uint16_t frequency, uint16_t duration) {
    _frequency = frequency;
    tone(_pin, _frequency);
    _active = true;
    _endTime = millis() + duration;
    _melody.clear();
}

void Buzzer::playMelody(const Melody& melody) {
    if (melody.empty()) return;

    _melody = melody;
    _melodyIndex = 0;
    _frequency = _melody[0].frequencyHz;

    if (_frequency == 0) noTone(_pin);
    else tone(_pin, _frequency);

    _active = true;
    _endTime = millis() + _melody[0].durationMs;
}

void Buzzer::update() {
    if (!_active) return;

    unsigned long now = millis();

    if (!_melody.empty() && now >= _endTime) {
        _melodyIndex++;
        if (_melodyIndex >= _melody.size()) {
            off();
            return;
        }

        _frequency = _melody[_melodyIndex].frequencyHz;

        if (_frequency == 0) noTone(_pin); // pause
        else tone(_pin, _frequency);

        _endTime = now + _melody[_melodyIndex].durationMs;
    } else if (_melody.empty() && _endTime != 0 && now >= _endTime) {
        off();
    }
}

} // namespace hardware
