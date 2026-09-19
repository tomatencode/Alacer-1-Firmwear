#pragma once
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <cstdint>
#include <functional>
#include <span>
#include <etl/string.h>
#include <optional>

#include "./Radio.hpp"

namespace hardware {

class HC12 : public Radio {
public:

    HC12(int setPin,
         int rxPin,
         int txPin
        );

    void begin();
    
    bool send(std::span<const uint8_t> data) override;
    bool available() override;
    std::optional<uint8_t> read() override;

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

    SoftwareSerial _serial;

    ATState _atState;
    uint32_t _atStartStepTime;
    uint32_t _atTimeout;
    etl::string<128> _atCommand;
    etl::string<128> _atResponse;
};

} // namespace hardware
