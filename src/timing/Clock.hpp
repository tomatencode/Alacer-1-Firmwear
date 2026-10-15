#pragma once

#include <Arduino.h>
#include <cstdint>

class Clock {
public:
    virtual ~Clock() = default;

    // Flight time defaults to real time.
    virtual uint32_t getTimeFlight_ms() const {
        return getTime_ms();
    }

    virtual uint32_t getTimeFlight_us() const {
        return getTime_us();
    }

    // Derived classes cannot override real time.
    virtual uint32_t getTime_ms() const final {
        return ::millis();
    }

    virtual uint32_t getTime_us() const final {
        return ::micros();
    }
};
