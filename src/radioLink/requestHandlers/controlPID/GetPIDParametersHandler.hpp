#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../controlPID/ControlPID.hpp"
#include "../../../helpers/codec/FixedPointCodec.hpp"

// GET_PID_PARAMETERS request:
//   empty payload.
// Response:
//   12 bytes: kp, ki, kd, each an int32 fixed-point value scaled by 100.
//   FAILURE with empty payload if the parameters are not configured yet.
class GetPIDParametersHandler {
public:
    explicit GetPIDParametersHandler(ControlPID& controlPID) : _controlPID(controlPID) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t>, std::span<uint8_t> response) {
        const std::optional<ControlPID::PIDParameters> parameters = _controlPID.getPIDParameters();
        if (!parameters.has_value()) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        fixedPoint::encode32(parameters->kp, response, 0);
        fixedPoint::encode32(parameters->ki, response, 4);
        fixedPoint::encode32(parameters->kd, response, 8);
        return {MessageScheduler::HandlerResultStatus::SUCCESS, 12};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<GetPIDParametersHandler, &GetPIDParametersHandler::handle>(*this);
    }

private:
    ControlPID& _controlPID;
};
