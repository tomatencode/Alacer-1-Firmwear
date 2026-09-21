#pragma once

#include "../MessageScheduler.hpp"
#include "../../locationEstimation/PositionAccumulator.hpp"
#include "./helpers/FixedPointCodec.hpp"

class GetPositionHandler : public MessageScheduler::RequestHandler {
public:
    explicit GetPositionHandler(PositionAccumulator& positionAccumulator)
        : _positionAccumulator(positionAccumulator) {}

    void handleRequest(std::span<const uint8_t>, MessageScheduler::HandlerResult resultCallback) override {
        Eigen::Vector3f position_m = _positionAccumulator.getPosition_m();
        uint8_t payload[12];
        std::span<uint8_t> buffer(payload);
        fixedPoint::encode32(position_m.x(), buffer, 0);
        fixedPoint::encode32(position_m.y(), buffer, 4);
        fixedPoint::encode32(position_m.z(), buffer, 8);
        resultCallback(MessageScheduler::HandlerResultStatus::SUCCESS, payload);
    }

private:
    PositionAccumulator& _positionAccumulator;
};