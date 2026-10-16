#pragma once
#include "timing/Clock.hpp"

#include <cstdint>
#include <optional>
#include "etl/delegate.h"

#include <Arduino.h>

#include "FlightTypes.hpp"
#include "FlightSequenceExecutor.hpp"

#include "../rotationEstimation/RotationAccumulator.hpp"
#include "../hardwareComponents/parachute/IParachute.hpp"
#include "../hardwareComponents/motor/IMotorIgniter.hpp"
#include "../hardwareIO/led/BlinkLed.hpp"

#include "../barometricHeightCalculation/BarometricHeightCalculator.hpp"
#include "../ascentTracking/VerticalMovementTracker.hpp"
#include "../ascentTracking/HorizontalMovementTracker.hpp"
#include "../controlPD/ControlPD.hpp"
#include "../logManagement/LogManager.hpp"


class FlightStateManager {
public:
    FlightStateManager(
        const Clock& clock,
        logging::LogManager& logManager,
        ControlPD& controlPID,
        RotationAccumulator& rotationAccumulator,
        BarometricHeightCalculator& barometricHeightCalculator,
        VerticalMovementTracker& verticalMovementTracker,
        HorizontalMovementTracker& horizontalMovementTracker,
        hardware::IMotorIgniter& motorIgniter,
        hardware::IParachute& parachute
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
    const Clock& _clock;
    FlightProfile _flightProfile;

    void changeState(FlightState newState) {
        _currentState = newState;
        _logManager.appendEvent(LogProtocol::FlightStateChangedEvent{.newState = newState});
    }

    FlightState _currentState = FlightState::IDLE;

    uint32_t _countdownStartTime = 0;
    uint32_t _motorStartBurnTime = 0;

    logging::LogManager& _logManager;
    FlightSequenceExecutor _executor;
};
