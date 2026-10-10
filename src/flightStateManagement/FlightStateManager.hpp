#pragma once

#include <cstdint>
#include <optional>
#include "etl/delegate.h"

#include <Arduino.h>

#include "FlightTypes.hpp"
#include "FlightSequenceExecutor.hpp"

#include "../rotationEstimation/RotationAccumulator.hpp"
#include "../hardwareComponents/parachute/Parachute.hpp"
#include "../hardwareComponents/motor/MotorIgniter.hpp"

#include "../barometricHeightCalculation/BarometricHeightCalculator.hpp"
#include "../ascentTracking/VerticalMovementTracker.hpp"
#include "../ascentTracking/HorizontalMovementTracker.hpp"
#include "../controlPID/ControlPID.hpp"
#include "../logManagement/LogManager.hpp"


class FlightStateManager {
public:
    FlightStateManager(
        LogManager& logManager,
        ControlPID& controlPID,
        RotationAccumulator& rotationAccumulator,
        BarometricHeightCalculator& barometricHeightCalculator,
        VerticalMovementTracker& verticalMovementTracker,
        HorizontalMovementTracker& horizontalMovementTracker,
        MotorIgniter& motorIgniter,
        Parachute& parachute
    );

    FlightState getCurrentState() const; // should not be used in logic, only for debugging

    // denied mid flight
    bool trySetIdle();

    // denied if not mid flight or already aborted
    bool abort();

    // denied if not in IDLE state
    bool startCountdown(FlightProfile flightProfile);

    bool retryDeployParachute();

    std::optional<uint32_t> getCountdownRemaining_ms() const;

    bool isMidFlight() const;

    void update();
private:
    FlightProfile _flightProfile;

    void changeState(FlightState newState) {
        _currentState = newState;
        _logManager.appendEvent(LogProtocol::FlightStateChangedEvent{.newState = newState});
    }

    FlightState _currentState = FlightState::IDLE;

    uint32_t _countdownStartTime = 0;
    uint32_t _motorStartBurnTime = 0;

    LogManager& _logManager;
    FlightSequenceExecutor _executor;
};
