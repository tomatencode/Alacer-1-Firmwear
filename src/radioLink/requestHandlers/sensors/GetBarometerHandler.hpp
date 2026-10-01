#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../hardwareIO/barometer/Barometer.hpp"
#include "../FixedPointCodec.hpp"

class GetBarometerHandler {
public:
    explicit GetBarometerHandler(hardware::Barometer& barometer) : _barometer(barometer) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t>, std::span<uint8_t> response) {
        fixedPoint::encode32(_barometer.getPressure_Pa(), response, 0);
        fixedPoint::encode32(_barometer.getTemperature_C(), response, 4);

        return {MessageScheduler::HandlerResultStatus::SUCCESS, 8};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<GetBarometerHandler, &GetBarometerHandler::handle>(*this);
    }

private:
    hardware::Barometer& _barometer;
};
