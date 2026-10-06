#pragma once

#include "../../MessageScheduler.hpp"
#include "../../../hardwareIO/battery/Battery.hpp"
#include "../../../helpers/codec/FixedPointCodec.hpp"

class GetBatteryVoltageHandler {
public:
    GetBatteryVoltageHandler(hardware::Battery& battery) : _battery(battery) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t>, std::span<uint8_t> response) {
        float voltage = _battery.getVoltage_v();
        fixedPoint::encode16(voltage, response, 0);
        return {MessageScheduler::HandlerResultStatus::SUCCESS, 2 };
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<GetBatteryVoltageHandler, &GetBatteryVoltageHandler::handle>(*this);
    }

private:
    hardware::Battery& _battery;
};