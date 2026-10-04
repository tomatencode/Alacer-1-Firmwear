#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../rotationEstimation/RotationAccumulator.hpp"

// SET_ACCUMULATING_ROTATION request:
//   payload[0] = 1 to start accumulating rotation from the IMU gyro,
//                0 to stop accumulating.
// Response:
//   SUCCESS with empty payload if the requested mode was applied.
//   FAILURE with empty payload if the payload is missing or payload[0] is not 0/1.
class SetAccumulatingRotationHandler {
public:
    explicit SetAccumulatingRotationHandler(RotationAccumulator& rotationAccumulator)
        : _rotationAccumulator(rotationAccumulator) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t>) {
        if (payload.size() < 1) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        if (payload[0] == 1) {
            _rotationAccumulator.startAccumulation();
        } else if (payload[0] == 0) {
            _rotationAccumulator.stopAccumulation();
        } else {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        return {MessageScheduler::HandlerResultStatus::SUCCESS, 0};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<SetAccumulatingRotationHandler, &SetAccumulatingRotationHandler::handle>(*this);
    }

private:
    RotationAccumulator& _rotationAccumulator;
};
