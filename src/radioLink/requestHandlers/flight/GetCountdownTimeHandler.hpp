#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../flightStateManagement/FlightStateManager.hpp"
#include "../LittleEndianCodec.hpp"

// GET_COUNTDOWN_TIME request:
//   empty payload.
// Response:
//   4 bytes, little-endian uint32 remaining countdown time in ms.
//   fails if the countdown is not active.
class GetCountdownTimeHandler {
public:
    explicit GetCountdownTimeHandler(FlightStateManager& flightStateManager)
        : _flightStateManager(flightStateManager) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t>, std::span<uint8_t> response) {
        const std::optional<uint32_t> remaining_ms_opt = _flightStateManager.getCountdownRemaining_ms();
        if (!remaining_ms_opt.has_value()) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }
        littleEndian::encodeU32(remaining_ms_opt.value(), response, 0);
        return {MessageScheduler::HandlerResultStatus::SUCCESS, 4};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<GetCountdownTimeHandler, &GetCountdownTimeHandler::handle>(*this);
    }

private:
    FlightStateManager& _flightStateManager;
};