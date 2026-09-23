#pragma once

#include <span>

#include "../MessageScheduler.hpp"
#include "../../hardwareIO/pyro/PyroChannel.hpp"

// FIRE_PYRO request:
//   payload[0] = channel index (0-based into the channels array given at construction)
//   payload[1..2] = optional fire duration in ms, little-endian uint16.
//                  If omitted, DEFAULT_FIRE_DURATION_MS is used.
// Response:
//   SUCCESS with empty payload if PyroChannel::fire() started.
//   FAILURE with empty payload if index out of range, duration invalid,
//   or channel refused to fire (no continuity / not armed).
class FirePyroHandler : public MessageScheduler::RequestHandler {
public:
    static constexpr uint32_t DEFAULT_FIRE_DURATION_MS = 1000;
    static constexpr uint32_t MAX_FIRE_DURATION_MS = 5000;

    explicit FirePyroHandler(std::span<PyroChannel*> channels) : _channels(channels) {}

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

        uint32_t durationMs = DEFAULT_FIRE_DURATION_MS;
        if (payload.size() >= 3) {
            durationMs = static_cast<uint32_t>(payload[1]) |
                         (static_cast<uint32_t>(payload[2]) << 8);
        }

        if (durationMs == 0 || durationMs > MAX_FIRE_DURATION_MS) {
            resultCallback(MessageScheduler::HandlerResultStatus::FAILURE, {});
            return;
        }

        const bool fired = _channels[channelIndex]->fire(durationMs);
        resultCallback(fired ? MessageScheduler::HandlerResultStatus::SUCCESS
                             : MessageScheduler::HandlerResultStatus::FAILURE,
                       {});
    }

private:
    std::span<PyroChannel*> _channels;
};
