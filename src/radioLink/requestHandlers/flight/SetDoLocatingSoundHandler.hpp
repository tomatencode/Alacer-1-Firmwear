#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../flightStateManagement/FlightStateManager.hpp"

namespace radioLink {

// SET_DO_LOCATING_SOUND request:
//   payload[0] = 1 to enable, 0 to disable.
// Response:
//   SUCCESS with empty payload if the setting was applied.
//   FAILURE with empty payload if the payload is missing or payload[0] is not 0/1.
class SetDoLocatingSoundHandler {
public:
    explicit SetDoLocatingSoundHandler(FlightStateManager& flightStateManager)
        : _flightStateManager(flightStateManager) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t>) {
        if (payload.size() < 1) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        if (payload[0] > 1) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        _flightStateManager.setDoLocatingSound(payload[0] == 1);
        return {MessageScheduler::HandlerResultStatus::SUCCESS, 0};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<
            SetDoLocatingSoundHandler, &SetDoLocatingSoundHandler::handle>(*this);
    }

private:
    FlightStateManager& _flightStateManager;
};

} // namespace radioLink
