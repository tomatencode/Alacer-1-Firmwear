#pragma once

#include "../MessageScheduler.hpp"
#include "../../rotationEstimation/RotationAccumulator.hpp"
#include "./helpers/FixedPointCodec.hpp"

class SetRotationHandler : public MessageScheduler::RequestHandler {
public:
    explicit SetRotationHandler(RotationAccumulator& rotationAccumulator)
        : _rotationAccumulator(rotationAccumulator) {}

    void handleRequest(std::span<const uint8_t> payload, MessageScheduler::HandlerResult resultCallback) override {
        if (payload.size() < 12) {
            resultCallback(MessageScheduler::HandlerResultStatus::FAILURE, {});
            return;
        }
        Eigen::Vector3f target_rad;
        target_rad.x() = fixedPoint::decode32(payload, 0);
        target_rad.y() = fixedPoint::decode32(payload, 4);
        target_rad.z() = fixedPoint::decode32(payload, 8);
        _rotationAccumulator.setEulerAngles_rad(target_rad);
        resultCallback(MessageScheduler::HandlerResultStatus::SUCCESS, {});
    }

private:
    RotationAccumulator& _rotationAccumulator;
};