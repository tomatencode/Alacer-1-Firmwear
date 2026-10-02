#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../ascentTracking/VerticalMovementTracker.hpp"
#include "../../../ascentTracking/HorizontalMovementTracker.hpp"
#include "../FixedPointCodec.hpp"

// Approximate flight location, taken from the ascent trackers.
//
// GET_FLIGHT_LOCATION request:
//   empty payload.
// Response (24 bytes, every field a fixed-point int32 scaled by 100):
//   [0..3]   horizontal x position (m, from HorizontalMovementTracker)
//   [4..7]   horizontal y position (m)
//   [8..11]  horizontal x velocity (m/s)
//   [12..15] horizontal y velocity (m/s)
//   [16..19] height above launch pad (m, from VerticalMovementTracker)
//   [20..23] vertical velocity (m/s)
// Velocity fields are only meaningful once the trackers have a velocity estimate.
class GetFlightLocationHandler {
public:
    explicit GetFlightLocationHandler(VerticalMovementTracker& verticalMovementTracker,
                                      HorizontalMovementTracker& horizontalMovementTracker)
        : _verticalMovementTracker(verticalMovementTracker),
          _horizontalMovementTracker(horizontalMovementTracker) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t>, std::span<uint8_t> response) {
        const Eigen::Vector2f position = _horizontalMovementTracker.getTotalMovement_m();
        const Eigen::Vector2f horizontalVelocity = _horizontalMovementTracker.getVelocity_m_s();

        fixedPoint::encode32(position.x(), response, 0);
        fixedPoint::encode32(position.y(), response, 4);
        fixedPoint::encode32(horizontalVelocity.x(), response, 8);
        fixedPoint::encode32(horizontalVelocity.y(), response, 12);
        fixedPoint::encode32(_verticalMovementTracker.getHeight_m(), response, 16);
        fixedPoint::encode32(_verticalMovementTracker.getVelocity_m_s(), response, 20);

        return {MessageScheduler::HandlerResultStatus::SUCCESS, 24};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<GetFlightLocationHandler, &GetFlightLocationHandler::handle>(*this);
    }

private:
    VerticalMovementTracker& _verticalMovementTracker;
    HorizontalMovementTracker& _horizontalMovementTracker;
};