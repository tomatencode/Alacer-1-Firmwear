#pragma once

#include <cstdint>
#include <optional>

#include <Arduino.h>

#include "../hardwareIO/pyro/PyroManager.hpp"

#include "../rotationEstimation/RotationAccumulator.hpp"
#include "../hardwareComponents/parachute/Parachute.hpp"
#include "../hardwareComponents/motor/MotorIgniter.hpp"

#include "../barometricHeightCalculation/BarometricHeightCalculator.hpp"
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

struct FlightProfile {
    uint32_t countdownDuration_ms;
    uint32_t motorBurnDuration_ms;

    Eigen::Quaternionf initialRotation;
    Eigen::Quaternionf targetAngle;

    float pidKp;
    float pidKi;
    float pidKd;

    int motorIgniterChannel;
    int parachutePyroChannel;

    float initialHeight_m;
};

class FlightStateManager {
public:
    FlightStateManager(ControlPID& controlPID, RotationAccumulator& rotationAccumulator, BarometricHeightCalculator& barometricHeightCalculator, VerticalMovementTracker& verticalMovementTracker, HorizontalMovementTracker& horizontalMovementTracker, PyroManager& pyroManager, MotorIgniter& motorIgniter, Parachute& parachute);

    FlightState getCurrentState() const; // should not be used in logic, only for debugging

    // denied mid flight
    bool trySetIdle();

    // denied if not mid flight or already aborted
    bool abort();

    // denied if not in IDLE state
    bool startCountdown(FlightProfile flightProfile);

    std::optional<uint32_t> getCountdownRemaining_ms() const;

    bool isMidFlight() const;

    void update();
private:
    bool _hasFlightProfile = false;
    FlightProfile _flightProfile;

    FlightState _currentState = FlightState::IDLE;

    uint32_t _countdownStartTime = 0;
    uint32_t _motorStartBurnTime;

    bool preflightChecks();

    bool configureForFlight();

    bool launchSequence();

    ControlPID& _controlPID;
    RotationAccumulator& _rotationAccumulator;
    BarometricHeightCalculator& _barometricHeightCalculator;
    VerticalMovementTracker& _verticalMovementTracker;
    HorizontalMovementTracker& _horizontalMovementTracker;
    PyroManager& _pyroManager;
    MotorIgniter& _motorIgniter;
    Parachute& _parachute;
};