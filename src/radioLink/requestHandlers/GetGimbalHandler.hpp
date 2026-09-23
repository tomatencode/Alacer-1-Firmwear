#pragma once

#include "../MessageScheduler.hpp"
#include "../../hardwareComponents/gimbal/Gimbal.hpp"
#include "./helpers/FixedPointCodec.hpp"

class GetGimbalHandler : public MessageScheduler::RequestHandler {
public:
    explicit GetGimbalHandler(hardware::Gimbal& gimbal) : _gimbal(gimbal) {}

    void handleRequest(std::span<const uint8_t>, MessageScheduler::HandlerResult resultCallback) override {
        hardware::Gimbal::GimbalPos pos = _gimbal.getCurrentPos();
        uint8_t payload[4];
        std::span<uint8_t> buffer(payload);
        fixedPoint::encode16(pos.pitch_deg, buffer, 0);
        fixedPoint::encode16(pos.yaw_deg, buffer, 2);
        resultCallback(MessageScheduler::HandlerResultStatus::SUCCESS, payload);
    }

private:
    hardware::Gimbal& _gimbal;
};