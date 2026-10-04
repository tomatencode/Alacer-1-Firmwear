#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../controlPID/ControlPID.hpp"
#include "../QuaternionCodec.hpp"

// GET_PID_TARGET request:
//   empty payload.
// Response:
//   16 bytes: target attitude quaternion (x, y, z, w), each an int32 fixed-point
//   value scaled by 100.
//   FAILURE with empty payload if no target is configured yet.
class GetPIDTargetHandler {
public:
    explicit GetPIDTargetHandler(ControlPID& controlPID) : _controlPID(controlPID) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t>, std::span<uint8_t> response) {
        const std::optional<Eigen::Quaternionf> target = _controlPID.getTarget();
        if (!target.has_value()) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        quaternionCodec::encode(target.value(), response, 0);
        return {MessageScheduler::HandlerResultStatus::SUCCESS, quaternionCodec::ENCODED_SIZE};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<GetPIDTargetHandler, &GetPIDTargetHandler::handle>(*this);
    }

private:
    ControlPID& _controlPID;
};
