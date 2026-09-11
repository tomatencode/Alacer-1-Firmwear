#pragma once

#include <Arduino.h>
#include <etl/vector.h>

class Buzzer {
public:
    struct Note {
        uint16_t frequencyHz; // 0 = pause
        uint16_t durationMs;
    };

    using Melody = etl::vector<Note, 32>;

    Buzzer(uint8_t pin);

    void begin();
    void on(uint16_t frequency = 1000); 
    void off();
    void beep(uint16_t frequency, uint16_t duration);

    void playMelody(const Melody& melody);
    void update();

private:
    uint8_t _pin;
    bool _active;
    uint16_t _frequency;
    unsigned long _endTime;

    Melody _melody;
    size_t _melodyIndex;
};