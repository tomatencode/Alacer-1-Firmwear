#pragma once

#include <span>
#include <etl/delegate.h>

#include "../../MessageScheduler.hpp"
#include "../../../flightStateManagement/FlightStateManager.hpp"
#include "../../../helpers/codec/LittleEndianCodec.hpp"
#include "../../../helpers/codec/FixedPointCodec.hpp"
#include "../../../helpers/codec/QuaternionCodec.hpp"

namespace radioLink {

// START_COUNTDOWN request:
//   payload (58 bytes):
//   [0..3]   countdown duration in ms, little-endian uint32
//   [4..7]   motor burn duration in ms, little-endian uint32
//   [8..23]  initial rotation, quaternion (x, y, z, w), each an int32 fixed-point
//            value scaled by 100 (fixedPoint::kScale)
//   [24..39] target attitude, quaternion (x, y, z, w), same encoding as above
//   [40..43] PID kp, int32 fixed-point scaled by 100
//   [44..47] PID kd, int32 fixed-point scaled by 100
//   [48]     motor igniter pyro channel index (uint8)
//   [49]     parachute pyro channel index (uint8)
//   [50..53] initial height in metres, int32 fixed-point scaled by 100
// Response:
//   SUCCESS with empty payload if FlightStateManager::startCountdown() accepted
//   the profile (state was IDLE and configuration/preflight checks passed).
//   FAILURE with empty payload if the payload is too short or the countdown
//   was refused.
class StartCountdownHandler {
public:
    static constexpr size_t PAYLOAD_SIZE = 58;

    // Application-owned binding keeps hardware configuration out of flight logic.
    // Simulation wiring can validate the indices without binding real channels.
    using ConfigureChannels = etl::delegate<bool(uint8_t, uint8_t)>;

    explicit StartCountdownHandler(FlightStateManager& flightStateManager, ConfigureChannels configureChannels)
        : _flightStateManager(flightStateManager), _configureChannels(configureChannels) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t>) {
        if (payload.size() < PAYLOAD_SIZE ||
            _flightStateManager.getCurrentState() != FlightState::IDLE ||
            !_configureChannels.is_valid()) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        FlightProfile flightProfile;
        flightProfile.countdownDuration_ms = littleEndian::decodeU32(payload, 0);
        flightProfile.motorBurnDuration_ms = littleEndian::decodeU32(payload, 4);
        flightProfile.initialRotation = quaternionCodec::decode(payload, 8);
        flightProfile.targetAngle = quaternionCodec::decode(payload, 24);
        flightProfile.pidKp = fixedPoint::decode32(payload, 40);
        flightProfile.pidKd = fixedPoint::decode32(payload, 44);
        flightProfile.initialHeight_m = fixedPoint::decode32(payload, 50);

        if (!_configureChannels(payload[48], payload[49])) {
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
