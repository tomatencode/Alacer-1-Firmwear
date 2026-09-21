#pragma once

#include "../MessageScheduler.hpp"
#include "../../locationEstimation/PositionAccumulator.hpp"
#include "./helpers/FixedPointCodec.hpp"

class SetPositionHandler : public MessageScheduler::RequestHandler {
public:
    explicit SetPositionHandler(PositionAccumulator& positionAccumulator)
        : _positionAccumulator(positionAccumulator) {}

    void handleRequest(std::span<const uint8_t> payload, MessageScheduler::HandlerResult resultCallback) override {
        if (payload.size() < 12) {
            resultCallback(MessageScheduler::HandlerResultStatus::FAILURE, {});
            return;
        }
        Eigen::Vector3f target_m;
        target_m.x() = fixedPoint::decode32(payload, 0);
        target_m.y() = fixedPoint::decode32(payload, 4);
        target_m.z() = fixedPoint::decode32(payload, 8);
        _positionAccumulator.setPosition_m_Velocity_m_s(target_m, Eigen::Vector3f::Zero());
        resultCallback(MessageScheduler::HandlerResultStatus::SUCCESS, {});
    }

private:
    PositionAccumulator& _positionAccumulator;
};