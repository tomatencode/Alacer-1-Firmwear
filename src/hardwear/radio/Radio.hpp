#pragma once
#include <Arduino.h>
#include <cstdint>
#include <span>
#include <optional>


class Radio {
public:
    virtual void begin() = 0;
    
    virtual void send(std::span<const uint8_t> data) = 0;
    virtual bool available() = 0;
    virtual std::optional<uint8_t> read() = 0;
};