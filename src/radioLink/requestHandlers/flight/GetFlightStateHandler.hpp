#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../flightStateManagement/FlightStateManager.hpp"

// GET_FLIGHT_STATE request:
//   empty payload.
// Response:
//   1 byte, the FlightState enum value:
//     0 = IDLE, 1 = COUNTDOWN, 2 = BURNING, 3 = COASTING,
//     4 = DESCENDING, 5 = LANDED, 6 = ABORTED
class GetFlightStateHandler {
public:
    explicit GetFlightStateHandler(FlightStateManager& flightStateManager)
        : _flightStateManager(flightStateManager) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t>, std::span<uint8_t> response) {
        response[0] = static_cast<uint8_t>(_flightStateManager.getCurrentState());
        return {MessageScheduler::HandlerResultStatus::SUCCESS, 1};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<GetFlightStateHandler, &GetFlightStateHandler::handle>(*this);
    }

private:
    FlightStateManager& _flightStateManager;
};