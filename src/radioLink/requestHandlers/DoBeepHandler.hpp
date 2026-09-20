#pragma once

#include "../MessageScheduler.hpp"
#include "../../hardware/buzzer/Buzzer.hpp"

class DoBeepHandler : public MessageScheduler::RequestHandler {
public:
    DoBeepHandler(hardware::Buzzer& buzzer) : _buzzer(buzzer) {}

    void handleRequest(std::span<const uint8_t> payload, MessageScheduler::HandlerResult resultCallback) override {
        _buzzer.beep(1000, 200);
        resultCallback(MessageScheduler::HandlerResultStatus::SUCCESS, std::span<const uint8_t>{});
    }

private:
    hardware::Buzzer& _buzzer;
};