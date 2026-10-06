#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../controlPID/ControlPID.hpp"
#include "../../../helpers/codec/FixedPointCodec.hpp"

// SET_PID_TARGET request:
//   payload (16 bytes): target attitude quaternion (x, y, z, w), each an int32
//   fixed-point value scaled by 100 (fixedPoint::kScale).
// Response:
//   SUCCESS with empty payload if the target was stored.
//   FAILURE with empty payload if the payload is shorter than 16 bytes.
class SetPIDTargetHandler {
public:
    explicit SetPIDTargetHandler(ControlPID& controlPID) : _controlPID(controlPID) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t>) {
        if (payload.size() < quaternionCodec::ENCODED_SIZE) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        _controlPID.setTarget(quaternionCodec::decode(payload, 0));
        return {MessageScheduler::HandlerResultStatus::SUCCESS, 0};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<SetPIDTargetHandler, &SetPIDTargetHandler::handle>(*this);
    }

private:
    ControlPID& _controlPID;
};
