#pragma once

#include <Arduino.h>

namespace hardware
{
    
    class Battery {
    public:
        Battery(int sensPin) : _sensPin(sensPin) {}

        void begin() {
            pinMode(_sensPin, INPUT_ANALOG);
        }

        float getVoltage_v() const {
            return analogRead(_sensPin) * (VOLTAGE_DIVIDER_RATIO * 3.3f / 1023.0f);
        }
    private:
        static constexpr float VOLTAGE_DIVIDER_RATIO = 57.0f / 47.0f;

        int _sensPin;
    };

} // namespace hardware