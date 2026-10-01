#pragma once

#include <span>

#include "../MessageScheduler.hpp"
#include "../../rotationEstimation/RotationAccumulator.hpp"
#include "./helpers/FixedPointCodec.hpp"

class SetRotationHandler {
public:
    explicit SetRotationHandler(RotationAccumulator& rotationAccumulator)
        : _rotationAccumulator(rotationAccumulator) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t>) {
        if (payload.size() < 12) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }
        Eigen::Vector3f target_rad;
        target_rad.x() = fixedPoint::decode32(payload, 0);
        target_rad.y() = fixedPoint::decode32(payload, 4);
        target_rad.z() = fixedPoint::decode32(payload, 8);
        _rotationAccumulator.setEulerAngles_rad(target_rad);
        return {MessageScheduler::HandlerResultStatus::SUCCESS, 0};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<SetRotationHandler, &SetRotationHandler::handle>(*this);
    }

private:
    RotationAccumulator& _rotationAccumulator;
};
