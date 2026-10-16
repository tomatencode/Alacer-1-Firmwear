#pragma once
#include "timing/Clock.hpp"

#include <Arduino.h>
#include <etl/vector.h>

namespace hardware {

class Buzzer {
public:
    struct Note {
        uint16_t frequencyHz; // 0 = pause
        uint16_t durationMs;
    };

    using Melody = etl::vector<Note, 32>;

    Buzzer(const Clock& clock, uint8_t pin);

    void begin();
    void on(uint16_t frequency = 1000); 
    void off();
    void beep(uint16_t frequency, uint16_t duration);

    void playMelody(const Melody& melody);
    void update();

private:
    const Clock& _clock;
    uint8_t _pin;
    bool _active;
    uint16_t _frequency;
    unsigned long _endTime;

    Melody _melody;
    size_t _melodyIndex;
};

} // namespace hardware