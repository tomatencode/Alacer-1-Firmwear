#pragma once

#include <algorithm>
#include <cstdint>
#include "simMode/timing/SimClock.hpp"

// One bounded step per firmware loop, so control/sensors run between physics steps.
class SimulationStepper {
public:
    explicit SimulationStepper(SimClock& clock, uint32_t speed = 1)
        : _clock(clock), _speed(speed) {}

    void begin() {
        _lastReal_us = _clock.getTime_us();
        _pending_us = 0;
    }

    void update() {
        const auto now = _clock.getTime_us();
        _pending_us += static_cast<uint64_t>(now - _lastReal_us) * _speed;
        _lastReal_us = now;
        const auto step = static_cast<uint32_t>(std::min(_pending_us, uint64_t{1000}));
        _pending_us -= step;
        _clock.advanceTimeBy_us(step);
    }

private:
    SimClock& _clock;
    uint32_t _speed;
    uint32_t _lastReal_us = 0;
    uint64_t _pending_us = 0;
};