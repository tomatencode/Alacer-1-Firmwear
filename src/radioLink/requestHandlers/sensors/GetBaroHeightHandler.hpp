#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../barometricHeightCalculation/BarometricHeightCalculator.hpp"
#include "../FixedPointCodec.hpp"

// GET_BARO_HEIGHT request:
//   empty payload.
// Response:
//   4 bytes, fixed-point int32 scaled by 100: height above the calibration
//   point (launch pad) in metres.
//   fails if the height is not available.
class GetBaroHeightHandler {
public:
    explicit GetBaroHeightHandler(BarometricHeightCalculator& heightCalculator)
        : _heightCalculator(heightCalculator) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t>, std::span<uint8_t> response) {
        const std::optional<float> height_m_opt = _heightCalculator.getHeight_m();
        if (!height_m_opt.has_value()) { return {MessageScheduler::HandlerResultStatus::FAILURE, 0}; }
        const float height_m = height_m_opt.value();

        fixedPoint::encode32(height_m, response, 0);
        return {MessageScheduler::HandlerResultStatus::SUCCESS, 4};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<GetBaroHeightHandler, &GetBaroHeightHandler::handle>(*this);
    }

private:
    BarometricHeightCalculator& _heightCalculator;
};