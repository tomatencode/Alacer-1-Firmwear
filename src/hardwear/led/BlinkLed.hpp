#pragma once
#include <cstdint>

namespace hardware {

class BlinkLed {
public:
    BlinkLed(int pin, uint8_t brightness, uint32_t defaultOnDuration_ms = 100);

    void begin();

    void update();
    void flash();
    void flash(uint32_t duration_ms);
private:
    int _pin;
    uint32_t _defaultOnDuration_ms;
    uint8_t _brightness;

    uint32_t _flashtime = 0;
    uint32_t _flashDuration_ms = 0;
};

} // namespace hardware