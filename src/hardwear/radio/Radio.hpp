#pragma once
#include <Arduino.h>
#include <cstdint>
#include <span>
#include <optional>

namespace hardware {

class Radio {
public:
    virtual void begin() = 0;
    
    virtual bool send(std::span<const uint8_t> data) = 0;
    virtual bool available() = 0;
    virtual std::optional<uint8_t> read() = 0;
};

} // namespace hardware