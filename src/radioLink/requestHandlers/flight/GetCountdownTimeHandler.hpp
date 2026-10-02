#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../stateManagement/FlightStateManager.hpp"
#include "../LittleEndianCodec.hpp"

// GET_COUNTDOWN_TIME request:
//   empty payload.
// Response:
//   4 bytes, little-endian uint32 remaining countdown time in ms.
//   0 when the flight is not counting down.
class GetCountdownTimeHandler {
public:
    explicit GetCountdownTimeHandler(FlightStateManager& flightStateManager)
        : _flightStateManager(flightStateManager) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t>, std::span<uint8_t> response) {
        littleEndian::encodeU32(_flightStateManager.getCountdownRemaining_ms(), response, 0);
        return {MessageScheduler::HandlerResultStatus::SUCCESS, 4};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<GetCountdownTimeHandler, &GetCountdownTimeHandler::handle>(*this);
    }

private:
    FlightStateManager& _flightStateManager;
};