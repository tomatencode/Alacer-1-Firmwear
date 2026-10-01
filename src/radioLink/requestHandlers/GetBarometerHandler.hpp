#pragma once

#include "../MessageScheduler.hpp"
#include "../../hardwareIO/barometer/Barometer.hpp"
#include "./helpers/FixedPointCodec.hpp"

class GetBarometerHandler : public MessageScheduler::RequestHandler {
public:
    explicit GetBarometerHandler(hardware::Barometer& barometer) : _barometer(barometer) {}

    void handleRequest(std::span<const uint8_t>, MessageScheduler::HandlerResult resultCallback) override {
        uint8_t payload[8];
        std::span<uint8_t> buffer(payload);
        fixedPoint::encode32(_barometer.getPressure_Pa(), buffer, 0);
        fixedPoint::encode32(_barometer.getTemperature_C(), buffer, 4);

        resultCallback(MessageScheduler::HandlerResultStatus::SUCCESS, payload);
    }

private:
    hardware::Barometer& _barometer;
};