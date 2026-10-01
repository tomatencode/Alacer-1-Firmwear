#pragma once

#include <span>

#include "../MessageScheduler.hpp"
#include "../../hardwareComponents/gimbal/Gimbal.hpp"
#include "./helpers/FixedPointCodec.hpp"

class SetGimbalHandler {
public:
    explicit SetGimbalHandler(hardware::Gimbal& gimbal) : _gimbal(gimbal) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t>) {
        if (payload.size() < 4) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }
        hardware::Gimbal::GimbalPos target;
        target.pitch_deg = fixedPoint::decode16(payload, 0);
        target.yaw_deg = fixedPoint::decode16(payload, 2);
        bool success = _gimbal.setTarget(target);
        return {success ? MessageScheduler::HandlerResultStatus::SUCCESS : MessageScheduler::HandlerResultStatus::FAILURE, 0};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<SetGimbalHandler, &SetGimbalHandler::handle>(*this);
    }

private:
    hardware::Gimbal& _gimbal;
};
