#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../barometricHeightCalculation/BarometricHeightCalculator.hpp"
#include "../../../helpers/codec/FixedPointCodec.hpp"

// CALIBRATE_BARO_HEIGHT request:
//   payload[0..3] = fixed-point int32 scaled by 100: the height the current
//   measurement should represent, in metres (e.g. 0 on the launch pad).
// Response:
//   SUCCESS with empty payload if the new reference was applied.
//   FAILURE with empty payload if the payload is missing/too short or the
//   barometer has not produced a valid measurement yet.
class CalibrateBaroHeightHandler {
public:
    explicit CalibrateBaroHeightHandler(BarometricHeightCalculator& heightCalculator)
        : _heightCalculator(heightCalculator) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t>) {
        if (payload.size() < 4) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        const float height_m = fixedPoint::decode32(payload, 0);
        const bool applied = _heightCalculator.calibrateTo(height_m);
        return {applied ? MessageScheduler::HandlerResultStatus::SUCCESS
                        : MessageScheduler::HandlerResultStatus::FAILURE,
                0};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<CalibrateBaroHeightHandler, &CalibrateBaroHeightHandler::handle>(*this);
    }

private:
    BarometricHeightCalculator& _heightCalculator;
};