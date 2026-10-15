#pragma once

#include "timing/Clock.hpp"

class SimClock : public Clock {
public:
    uint32_t getTimeFlight_ms() const override {
        return static_cast<uint32_t>(currentTime_us / 1000);
    }

    uint32_t getTimeFlight_us() const override {
        return static_cast<uint32_t>(currentTime_us);
    }

    void advanceTimeBy_ms(uint32_t delta_ms) {
        currentTime_us += static_cast<uint64_t>(delta_ms) * 1000;
    }

    void advanceTimeBy_us(uint32_t delta_us) {
        currentTime_us += delta_us;
    }

private:
    uint64_t currentTime_us = 0;
};
