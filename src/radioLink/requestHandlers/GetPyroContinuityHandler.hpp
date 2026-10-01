#pragma once

#include <span>

#include "../MessageScheduler.hpp"
#include "../../hardwareIO/pyro/PyroChannel.hpp"

// GET_PYRO_CONTINUITY request:
//   payload[0] = channel index (0-based into the channels array given at construction)
// Response:
//   SUCCESS with payload[0] = 1 if channel has continuity, 0 otherwise.
//   FAILURE with empty payload if index out of range.
class GetPyroContinuityHandler {
public:
    explicit GetPyroContinuityHandler(std::span<PyroChannel*> channels) : _channels(channels) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t> response) {
        if (payload.size() < 1) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        const uint8_t channelIndex = payload[0];
        if (channelIndex >= _channels.size() || _channels[channelIndex] == nullptr) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        response[0] = _channels[channelIndex]->hasContinuity() ? 1 : 0;
        return {MessageScheduler::HandlerResultStatus::SUCCESS, 1};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<GetPyroContinuityHandler, &GetPyroContinuityHandler::handle>(*this);
    }

private:
    std::span<PyroChannel*> _channels;
};
