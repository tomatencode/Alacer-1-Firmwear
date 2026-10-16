#pragma once

#include <span>
#include <etl/delegate.h>

#include "../../MessageScheduler.hpp"
#include "../../../flightStateManagement/FlightStateManager.hpp"
#include "../../../helpers/codec/FixedPointCodec.hpp"
#include "../../../helpers/codec/QuaternionCodec.hpp"
#include "../../../helpers/codec/StringCodec.hpp"

namespace radioLink {

// START_COUNTDOWN request:
//   [0..N]   length-prefixed log filename (1..32 bytes including the length byte)
//   [N..N+15] initial rotation, quaternion (x, y, z, w), each an int32 fixed-point
//            value scaled by 100 (fixedPoint::kScale)
//   [N+16..N+19] PID kp, int32 fixed-point scaled by 100
//   [N+20..N+23] PID kd, int32 fixed-point scaled by 100
//   [N+24]  motor igniter pyro channel index (uint8)
//   [N+25]  parachute pyro channel index (uint8)
// Response:
//   SUCCESS with empty payload if FlightStateManager::startCountdown() accepted
//   the profile (state was IDLE and configuration/preflight checks passed).
//   FAILURE with empty payload if the payload is malformed or the countdown
//   was refused.
class StartCountdownHandler {
public:
    static constexpr size_t PROFILE_SIZE = quaternionCodec::ENCODED_SIZE + 2 * sizeof(int32_t) + 2;

    // Application-owned binding keeps hardware configuration out of flight logic.
    // Simulation wiring can validate the indices without binding real channels.
    using ConfigureChannels = etl::delegate<bool(uint8_t, uint8_t)>;

    explicit StartCountdownHandler(FlightStateManager& flightStateManager, ConfigureChannels configureChannels)
        : _flightStateManager(flightStateManager), _configureChannels(configureChannels) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t>) {
        FlightProfile flightProfile;
        const auto filenameSize = stringCodec::decode(payload, 0, flightProfile.name);
        if (!filenameSize || flightProfile.name.empty() ||
            payload.size() != *filenameSize + PROFILE_SIZE ||
            _flightStateManager.getCurrentState() != FlightState::IDLE ||
            !_configureChannels.is_valid()) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        const size_t offset = *filenameSize;
        flightProfile.initialRotation = quaternionCodec::decode(payload, offset);
        flightProfile.pidKp = fixedPoint::decode32(payload, offset + quaternionCodec::ENCODED_SIZE);
        flightProfile.pidKd = fixedPoint::decode32(payload, offset + quaternionCodec::ENCODED_SIZE + sizeof(int32_t));

        if (!_configureChannels(payload[offset + PROFILE_SIZE - 2], payload[offset + PROFILE_SIZE - 1])) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        const bool started = _flightStateManager.startCountdown(flightProfile);
        return {started ? MessageScheduler::HandlerResultStatus::SUCCESS
                        : MessageScheduler::HandlerResultStatus::FAILURE,
                0};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<StartCountdownHandler, &StartCountdownHandler::handle>(*this);
    }

private:
    FlightStateManager& _flightStateManager;
    ConfigureChannels _configureChannels;
};

} // namespace radioLink
