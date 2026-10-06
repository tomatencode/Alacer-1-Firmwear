#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../rotationEstimation/RotationAccumulator.hpp"
#include "../../../helpers/codec/QuaternionCodec.hpp"

// GET_ROTATION request:
//   empty payload.
// Response:
//   16 bytes: current orientation quaternion (x, y, z, w), each an int32
//   fixed-point value scaled by 100 (fixedPoint::kScale).
class GetRotationHandler {
public:
    explicit GetRotationHandler(RotationAccumulator& rotationAccumulator)
        : _rotationAccumulator(rotationAccumulator) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t>, std::span<uint8_t> response) {
        quaternionCodec::encode(_rotationAccumulator.getRotationQuaternion(), response, 0);
        return {MessageScheduler::HandlerResultStatus::SUCCESS, quaternionCodec::ENCODED_SIZE};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<GetRotationHandler, &GetRotationHandler::handle>(*this);
    }

private:
    RotationAccumulator& _rotationAccumulator;
};
