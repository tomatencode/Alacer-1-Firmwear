#pragma once
#include <Arduino.h>
#include <cstdint>
#include <functional>
#include <span>
#include <etl/string.h>
#include <optional>


class HC12 {
public:

    HC12(int setPin,
         int rxPin,
         int txPin,
         int baudRate = 9600,
         std::function<void()> onSendCallback = nullptr,
         std::function<void()> onReceiveCallback = nullptr
        );

    void begin();
    
    bool send(std::span<const uint8_t> data);
    bool available();
    std::optional<uint8_t> read();

    bool sendATCommand(const char* command, uint32_t timeout_ms = 200);
    bool atBusy();
    bool atDone();
    std::optional<etl::string<128>> takeATResponse();

    void update();

private:
    enum class ATState { IDLE, ENTERING_AT_MODE, AWAITING_RESPONSE, EXITING_AT_MODE, DONE, };

    int _setPin;
    int _rxPin;
    int _txPin;

    int _baudRate;

    HardwareSerial _serial;

    std::function<void()> _onSendCallback;
    std::function<void()> _onReceiveCallback;

    ATState _atState;
    uint32_t _atStartStepTime;
    uint32_t _atTimeout;
    etl::string<128> _atCommand;
    etl::string<128> _atResponse;
};
