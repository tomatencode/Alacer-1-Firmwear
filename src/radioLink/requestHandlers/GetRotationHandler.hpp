#pragma once

#include <span>

#include "../MessageScheduler.hpp"
#include "../../rotationEstimation/RotationAccumulator.hpp"
#include "./helpers/FixedPointCodec.hpp"

class GetRotationHandler {
public:
    explicit GetRotationHandler(RotationAccumulator& rotationAccumulator)
        : _rotationAccumulator(rotationAccumulator) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t>, std::span<uint8_t> response) {
        Eigen::Vector3f rotation_rad = _rotationAccumulator.getEulerAngles_rad();
        fixedPoint::encode32(rotation_rad.x(), response, 0);
        fixedPoint::encode32(rotation_rad.y(), response, 4);
        fixedPoint::encode32(rotation_rad.z(), response, 8);
        return {MessageScheduler::HandlerResultStatus::SUCCESS, 12};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<GetRotationHandler, &GetRotationHandler::handle>(*this);
    }

private:
    RotationAccumulator& _rotationAccumulator;
};
