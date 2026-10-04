#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../controlPID/ControlPID.hpp"
#include "../FixedPointCodec.hpp"

// SET_PID_PARAMETERS request:
//   payload (12 bytes): kp, ki, kd, each an int32 fixed-point value scaled by
//   100 (fixedPoint::kScale).
// Response:
//   SUCCESS with empty payload if the parameters were stored.
//   FAILURE with empty payload if the payload is shorter than 12 bytes.
class SetPIDParametersHandler {
public:
    static constexpr size_t PAYLOAD_SIZE = 12;

    explicit SetPIDParametersHandler(ControlPID& controlPID) : _controlPID(controlPID) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t>) {
        if (payload.size() < PAYLOAD_SIZE) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        _controlPID.setPIDParameters(
            fixedPoint::decode32(payload, 0),
            fixedPoint::decode32(payload, 4),
            fixedPoint::decode32(payload, 8));
        return {MessageScheduler::HandlerResultStatus::SUCCESS, 0};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<SetPIDParametersHandler, &SetPIDParametersHandler::handle>(*this);
    }

private:
    ControlPID& _controlPID;
};
