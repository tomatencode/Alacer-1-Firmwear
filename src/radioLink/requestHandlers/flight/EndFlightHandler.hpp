#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../flightStateManagement/FlightStateManager.hpp"

// END_FLIGHT request:
//   empty payload.
// Response:
//   SUCCESS with empty payload if FlightStateManager::trySetIdle() accepted
//   (COUNTDOWN, ABORTED or LANDED -> IDLE).
//   FAILURE with empty payload if denied (already IDLE or mid flight).
class EndFlightHandler {
public:
    explicit EndFlightHandler(FlightStateManager& flightStateManager)
        : _flightStateManager(flightStateManager) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t>, std::span<uint8_t>) {
        const bool setIdle = _flightStateManager.trySetIdle();
        return {setIdle ? MessageScheduler::HandlerResultStatus::SUCCESS
                        : MessageScheduler::HandlerResultStatus::FAILURE,
                0};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<EndFlightHandler, &EndFlightHandler::handle>(*this);
    }

private:
    FlightStateManager& _flightStateManager;
};