#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../stateManagement/FlightStateManager.hpp"

// ABORT_FLIGHT request:
//   empty payload.
// Response:
//   SUCCESS with empty payload if the abort was accepted.
//   FAILURE with empty payload if the current state refuses to abort
//   (IDLE, DESCENDING, LANDED or already ABORTED).
class AbortFlightHandler {
public:
    explicit AbortFlightHandler(FlightStateManager& flightStateManager)
        : _flightStateManager(flightStateManager) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t>, std::span<uint8_t>) {
        const bool aborted = _flightStateManager.abort();
        return {aborted ? MessageScheduler::HandlerResultStatus::SUCCESS
                        : MessageScheduler::HandlerResultStatus::FAILURE,
                0};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<AbortFlightHandler, &AbortFlightHandler::handle>(*this);
    }

private:
    FlightStateManager& _flightStateManager;
};