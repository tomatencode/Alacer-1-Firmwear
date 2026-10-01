#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../hardwareIO/buzzer/Buzzer.hpp"

class DoBeepHandler {
public:
    DoBeepHandler(hardware::Buzzer& buzzer) : _buzzer(buzzer) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t>, std::span<uint8_t>) {
        _buzzer.beep(1000, 200);
        return {MessageScheduler::HandlerResultStatus::SUCCESS, 0};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<DoBeepHandler, &DoBeepHandler::handle>(*this);
    }

private:
    hardware::Buzzer& _buzzer;
};
