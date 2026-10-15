#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../controlPD/ControlPD.hpp"
#include "../../../helpers/codec/FixedPointCodec.hpp"

namespace radioLink {

// SET_PID_PARAMETERS request:
//   payload (8 bytes): kp, kd, each an int32 fixed-point value scaled by
//   100 (fixedPoint::kScale).
// Response:
//   SUCCESS with empty payload if the parameters were stored.
//   FAILURE with empty payload if the payload is shorter than 8 bytes.
class SetPDParametersHandler {
public:
    static constexpr size_t PAYLOAD_SIZE = 8;

    explicit SetPDParametersHandler(ControlPD& controlPID) : _controlPID(controlPID) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t>) {
        if (payload.size() < PAYLOAD_SIZE) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        _controlPID.setPDParameters(
            fixedPoint::decode32(payload, 0),
            fixedPoint::decode32(payload, 4));
        return {MessageScheduler::HandlerResultStatus::SUCCESS, 0};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<SetPDParametersHandler, &SetPDParametersHandler::handle>(*this);
    }

private:
    ControlPD& _controlPID;
};

} // namespace radioLink
