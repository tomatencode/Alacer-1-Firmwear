#pragma once

#include <cstdint>

#include <Arduino.h>

#include "../rotationEstimation/RotationAccumulator.hpp"
#include "../hardwareComponents/parashoot/Parashoot.hpp"
#include "../hardwareComponents/motor/MotorIgniter.hpp"

#include "../acentTracking/VerticalMovementTracker.hpp"

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
    FlightStateManager(RotationAccumulator& rotationAccumulator, VerticalMovementTracker& verticalMovementTracker, MotorIgniter& motorIgniter, Parachute& parashoot);

    FlightState getCurrentState() const; // shuld not be used in logic, only for debugging

    // denyd mid flight
    bool trySetIdle();

    // denyd if not mid flight or already aborted
    bool abort();

    // denyd if not in IDLE state
    bool startCountdown();

    void update();
private:
    static constexpr uint32_t COUNTDOWN_DURATION_ms = 10000;
    static constexpr uint32_t MOTOR_BURN_DURATION_ms = 6500;


    FlightState _currentState = FlightState::IDLE;

    uint32_t _countdownStartTime;
    uint32_t _motorStartBurnTime;

    bool preflightChecks();

    void launch();

    RotationAccumulator& _rotationAccumulator;
    VerticalMovementTracker& _verticalMovementTracker;
    MotorIgniter& _motorIgniter;
    Parachute& _parashoot;
};