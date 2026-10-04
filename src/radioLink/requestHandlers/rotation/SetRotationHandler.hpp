#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../rotationEstimation/RotationAccumulator.hpp"
#include "../QuaternionCodec.hpp"

// SET_ROTATION request:
//   payload (16 bytes): orientation quaternion (x, y, z, w), each an int32
//   fixed-point value scaled by 100 (fixedPoint::kScale).
// Response:
//   SUCCESS with empty payload if the rotation was applied.
//   FAILURE with empty payload if the payload is shorter than 16 bytes.
class SetRotationHandler {
public:
    explicit SetRotationHandler(RotationAccumulator& rotationAccumulator)
        : _rotationAccumulator(rotationAccumulator) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t>) {
        if (payload.size() < quaternionCodec::ENCODED_SIZE) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        _rotationAccumulator.setRotationQuaternion(quaternionCodec::decode(payload, 0));
        return {MessageScheduler::HandlerResultStatus::SUCCESS, 0};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<SetRotationHandler, &SetRotationHandler::handle>(*this);
    }

private:
    RotationAccumulator& _rotationAccumulator;
};
