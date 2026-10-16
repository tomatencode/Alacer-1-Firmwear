#pragma once

#include <cstdint>

#include <ArduinoEigen.h>

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
    float pidKd;

    float initialHeight_m;
};
