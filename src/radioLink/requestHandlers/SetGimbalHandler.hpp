#pragma once

#include "../MessageScheduler.hpp"
#include "../../hardwareComponents/gimbal/Gimbal.hpp"
#include "./helpers/FixedPointCodec.hpp"

class SetGimbalHandler : public MessageScheduler::RequestHandler {
public:
    explicit SetGimbalHandler(hardware::Gimbal& gimbal) : _gimbal(gimbal) {}

    void handleRequest(std::span<const uint8_t> payload, MessageScheduler::HandlerResult resultCallback) override {
        if (payload.size() < 4) {
            resultCallback(MessageScheduler::HandlerResultStatus::FAILURE, {});
            return;
        }
        hardware::Gimbal::GimbalPos target;
        target.pitch_deg = fixedPoint::decode16(payload, 0);
        target.yaw_deg = fixedPoint::decode16(payload, 2);
        _gimbal.setTarget(target);
        resultCallback(MessageScheduler::HandlerResultStatus::SUCCESS, {});
    }

private:
    hardware::Gimbal& _gimbal;
};