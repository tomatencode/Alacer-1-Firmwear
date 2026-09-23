#pragma once

#include <cstdint>

#include <Arduino.h>

#include "../rotationEstimation/RotationAccumulator.hpp"

enum class FlightState {
    IDLE,
    COUNTDOWN,
    BURNING,
    COASTING,
    DESCENDING,
    LANDED,

    ABORTING,
    ABORTED
};

const uint32_t COUNTDOWN_DURATION = 10000;

class FlightStateManager {
public:
    FlightStateManager(RotationAccumulator& rotationAccumulator);

    FlightState getCurrentState() const; // shuld not be used in logic, only for debugging

    bool trySetIdle(); // denyd mid flight

    bool startAbort(); // denyd if not mid flight or already aborted
    bool isAbortDone();

    bool startCountdown(); // denyd if not in IDLE state

    void update();
private:
    FlightState _currentState = FlightState::IDLE;

    uint32_t _countdownStartTime;


    bool preflightChecks();

    RotationAccumulator& _rotationAccumulator;
};