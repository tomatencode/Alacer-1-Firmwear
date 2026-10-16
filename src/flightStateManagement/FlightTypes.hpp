#pragma once

#include <cstdint>
#include <etl/string.h>

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
    etl::string<32> name;

    float pidKp;
    float pidKd;

    Eigen::Quaternionf initialRotation;
};
