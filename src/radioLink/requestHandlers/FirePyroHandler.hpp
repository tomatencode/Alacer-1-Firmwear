#pragma once

#include <span>

#include "../MessageScheduler.hpp"
#include "../../hardwareIO/pyro/PyroChannel.hpp"
#include "./helpers/LittleEndianCodec.hpp"

// FIRE_PYRO request:
//   payload[0] = channel index (0-based into the channels array given at construction)
//   payload[1..2] = optional fire duration in ms, little-endian uint16.
//                  If omitted, DEFAULT_FIRE_DURATION_MS is used.
// Response:
//   SUCCESS with empty payload if PyroChannel::fire() started.
//   FAILURE with empty payload if index out of range, duration invalid,
//   or channel refused to fire (no continuity / not armed).
class FirePyroHandler {
public:
    static constexpr uint32_t DEFAULT_FIRE_DURATION_MS = 1000;
    static constexpr uint32_t MAX_FIRE_DURATION_MS = 5000;

    explicit FirePyroHandler(std::span<PyroChannel*> channels) : _channels(channels) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t>) {
        if (payload.size() < 1) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        const uint8_t channelIndex = payload[0];
        if (channelIndex >= _channels.size() || _channels[channelIndex] == nullptr) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        uint32_t durationMs = DEFAULT_FIRE_DURATION_MS;
        if (payload.size() >= 3) {
            durationMs = littleEndian::decodeU16(payload, 1);
        }

        if (durationMs == 0 || durationMs > MAX_FIRE_DURATION_MS) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        const bool fired = _channels[channelIndex]->fire(durationMs);
        return {fired ? MessageScheduler::HandlerResultStatus::SUCCESS
                      : MessageScheduler::HandlerResultStatus::FAILURE,
                0};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<FirePyroHandler, &FirePyroHandler::handle>(*this);
    }

private:
    std::span<PyroChannel*> _channels;
};
