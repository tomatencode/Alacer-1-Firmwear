#pragma once

#include <cstdint>

#include <ArduinoEigen.h>

#include "../hardwareIO/pyro/PyroChannel.hpp"

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

    PyroChannel* motorIgniterChannel;
    PyroChannel* parachutePyroChannel;

    float initialHeight_m;
};
