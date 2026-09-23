#pragma once

#include "../MessageScheduler.hpp"
#include "../../rotationEstimation/RotationAccumulator.hpp"
#include "./helpers/FixedPointCodec.hpp"

class GetRotationHandler : public MessageScheduler::RequestHandler {
public:
    explicit GetRotationHandler(RotationAccumulator& rotationAccumulator)
        : _rotationAccumulator(rotationAccumulator) {}

    void handleRequest(std::span<const uint8_t>, MessageScheduler::HandlerResult resultCallback) override {
        Eigen::Vector3f rotation_rad = _rotationAccumulator.getEulerAngles_rad();
        uint8_t payload[12];
        std::span<uint8_t> buffer(payload);
        fixedPoint::encode32(rotation_rad.x(), buffer, 0);
        fixedPoint::encode32(rotation_rad.y(), buffer, 4);
        fixedPoint::encode32(rotation_rad.z(), buffer, 8);
        resultCallback(MessageScheduler::HandlerResultStatus::SUCCESS, payload);
    }

private:
    RotationAccumulator& _rotationAccumulator;
};