#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../controlPID/ControlPID.hpp"

// SET_CONTROLLING request:
//   payload[0] = 1 to start controlling, 0 to stop controlling.
// Response:
//   SUCCESS with empty payload if the requested state was reached.
//   FAILURE with empty payload if the payload is missing, payload[0] is not
//   0/1, or starting was refused (PID parameters or target not configured).
class SetControllingHandler {
public:
    explicit SetControllingHandler(ControlPID& controlPID) : _controlPID(controlPID) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t>) {
        if (payload.size() < 1) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        bool reached;
        if (payload[0] == 1) {
            reached = _controlPID.setControlling(true);
        } else if (payload[0] == 0) {
            reached = _controlPID.setControlling(false);
        } else {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        return {reached ? MessageScheduler::HandlerResultStatus::SUCCESS
                        : MessageScheduler::HandlerResultStatus::FAILURE,
                0};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<SetControllingHandler, &SetControllingHandler::handle>(*this);
    }

private:
    ControlPID& _controlPID;
};
