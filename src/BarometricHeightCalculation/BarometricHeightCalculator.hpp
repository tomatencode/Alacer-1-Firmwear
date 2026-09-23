#pragma once

#include <cstdint>

#include "../hardware/barometer/Barometer.hpp"


class BarometricHeightCalculator {
public:
    explicit BarometricHeightCalculator(hardware::Barometer& barometer)
        : _barometer(barometer) {}

    float getHeight_m() const { return _height_m; }

    float getVelocity_m_s() const { return _velocity_m_s; }

    void calibrateTo(float height_m);

    void update();

private:
    hardware::Barometer& _barometer;

    float _height_m = 0.0f;
    float _velocity_m_s = 0.0f;

    bool _isCalibrated = false;
};