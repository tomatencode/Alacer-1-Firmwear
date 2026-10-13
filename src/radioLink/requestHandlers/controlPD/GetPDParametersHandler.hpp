#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../controlPD/ControlPD.hpp"
#include "../../../helpers/codec/FixedPointCodec.hpp"

// GET_PID_PARAMETERS request:
//   empty payload.
// Response:
//   8 bytes: kp, kd, each an int32 fixed-point value scaled by 100.
//   FAILURE with empty payload if the parameters are not configured yet.
class GetPDParametersHandler {
public:
    explicit GetPDParametersHandler(ControlPD& controlPID) : _controlPID(controlPID) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t>, std::span<uint8_t> response) {
        const std::optional<ControlPD::PDParameters> parameters = _controlPID.getPDParameters();
        if (!parameters.has_value()) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        fixedPoint::encode32(parameters->kp, response, 0);
        fixedPoint::encode32(parameters->kd, response, 4);
        return {MessageScheduler::HandlerResultStatus::SUCCESS, 8};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<GetPDParametersHandler, &GetPDParametersHandler::handle>(*this);
    }

private:
    ControlPD& _controlPID;
};
