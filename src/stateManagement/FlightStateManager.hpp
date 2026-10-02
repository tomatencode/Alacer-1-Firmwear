#pragma once

#include <cstdint>

#include <Arduino.h>

#include "../rotationEstimation/RotationAccumulator.hpp"
#include "../hardwareComponents/parachute/Parachute.hpp"
#include "../hardwareComponents/motor/MotorIgniter.hpp"

#include "../ascentTracking/VerticalMovementTracker.hpp"
#include "../ascentTracking/HorizontalMovementTracker.hpp"
#include "../controlPID/ControlPID.hpp"


enum class FlightState {
    IDLE,
    COUNTDOWN,
    BURNING,
    COASTING,
    DESCENDING,
    LANDED,

    ABORTED
};

class FlightStateManager {
public:
    FlightStateManager(ControlPID& controlPID, RotationAccumulator& rotationAccumulator, VerticalMovementTracker& verticalMovementTracker, HorizontalMovementTracker& horizontalMovementTracker, MotorIgniter& motorIgniter, Parachute& parachute);

    FlightState getCurrentState() const; // should not be used in logic, only for debugging

    // denied mid flight
    bool trySetIdle();

    // denied if not mid flight or already aborted
    bool abort();

    // denied if not in IDLE state
    bool startCountdown();

    // remaining COUNTDOWN time in ms, 0 when not counting down
    uint32_t getCountdownRemaining_ms() const;

    void update();
private:
    static constexpr uint32_t COUNTDOWN_DURATION_ms = 10000;
    static constexpr uint32_t MOTOR_BURN_DURATION_ms = 6500;


    FlightState _currentState = FlightState::IDLE;

    uint32_t _countdownStartTime = 0;
    uint32_t _motorStartBurnTime;

    bool preflightChecks();

    void launch();

    ControlPID& _controlPID;
    RotationAccumulator& _rotationAccumulator;
    VerticalMovementTracker& _verticalMovementTracker;
    HorizontalMovementTracker& _horizontalMovementTracker;
    MotorIgniter& _motorIgniter;
    Parachute& _parachute;
};