#pragma once

#include <span>

#include "../MessageScheduler.hpp"
#include "../../hardwareIO/pyro/PyroChannel.hpp"

// GET_PYRO_CONTINUITY request:
//   payload[0] = channel index (0-based into the channels array given at construction)
// Response:
//   SUCCESS with payload[0] = 1 if channel has continuity, 0 otherwise.
//   FAILURE with empty payload if index out of range.
class GetPyroContinuityHandler : public MessageScheduler::RequestHandler {
public:
    explicit GetPyroContinuityHandler(std::span<PyroChannel*> channels) : _channels(channels) {}

    void handleRequest(std::span<const uint8_t> payload, MessageScheduler::HandlerResult resultCallback) override {
        if (payload.size() < 1) {
            resultCallback(MessageScheduler::HandlerResultStatus::FAILURE, {});
            return;
        }

        const uint8_t channelIndex = payload[0];
        if (channelIndex >= _channels.size() || _channels[channelIndex] == nullptr) {
            resultCallback(MessageScheduler::HandlerResultStatus::FAILURE, {});
            return;
        }

        const uint8_t hasContinuity = _channels[channelIndex]->hasContinuity() ? 1 : 0;
        resultCallback(MessageScheduler::HandlerResultStatus::SUCCESS,
                       std::span<const uint8_t>(&hasContinuity, 1));
    }

private:
    std::span<PyroChannel*> _channels;
};
