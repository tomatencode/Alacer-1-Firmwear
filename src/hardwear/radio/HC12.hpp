#pragma once
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <optional>
#include "Radio.hpp"


class HC12 : public Radio {
public:
    HC12(int setPin,
         int rxPin,
         int txPin,
         int baudRate = 9600,
         std::function<void()> onSendCallback = nullptr,
         std::function<void()> onReceiveCallback = nullptr
        );

    void begin();
    
    void send(std::span<const uint8_t> data);
    bool available();
    std::optional<uint8_t> read();

    std::string sendATCommand(const char* command, uint32_t timeout_ms = 200);
private:
    int _setPin;
    int _rxPin;
    int _txPin;

    int _baudRate;

    SoftwareSerial _serial;

    std::function<void()> _onSendCallback;
    std::function<void()> _onReceiveCallback;
};
