#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../stateManagement/FlightStateManager.hpp"

// RETRY_DEPLOY_PARACHUTE request:
//   empty payload.
// Response:
//   SUCCESS with empty payload if FlightStateManager::retryDeployParachute()
//   accepted (state is ABORTED and the parachute pyro channel fired).
//   FAILURE with empty payload if denied (not ABORTED or deployment refused).
class RetryDeployParachuteHandler {
public:
    explicit RetryDeployParachuteHandler(FlightStateManager& flightStateManager)
        : _flightStateManager(flightStateManager) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t>, std::span<uint8_t>) {
        const bool deployed = _flightStateManager.retryDeployParachute();
        return {deployed ? MessageScheduler::HandlerResultStatus::SUCCESS
                         : MessageScheduler::HandlerResultStatus::FAILURE,
                0};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<RetryDeployParachuteHandler, &RetryDeployParachuteHandler::handle>(*this);
    }

private:
    FlightStateManager& _flightStateManager;
};
