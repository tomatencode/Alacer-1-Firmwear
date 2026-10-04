#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../controlPID/ControlPID.hpp"

// GET_CONTROLLING request (is controlling):
//   empty payload.
// Response:
//   SUCCESS with payload[0] = 1 if the PID controller is currently controlling,
//   0 otherwise.
class GetControllingHandler {
public:
    explicit GetControllingHandler(ControlPID& controlPID) : _controlPID(controlPID) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t>, std::span<uint8_t> response) {
        response[0] = _controlPID.isControlling() ? 1 : 0;
        return {MessageScheduler::HandlerResultStatus::SUCCESS, 1};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<GetControllingHandler, &GetControllingHandler::handle>(*this);
    }

private:
    ControlPID& _controlPID;
};
