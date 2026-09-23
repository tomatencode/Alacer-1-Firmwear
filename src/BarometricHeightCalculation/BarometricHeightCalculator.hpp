#pragma once

#include <cstddef>
#include <cstdint>

#include "../hardwareIO/barometer/Barometer.hpp"


// Turns the pressure measured by a barometer into height and vertical velocity.
//
// Height is calculated with the international barometric formula for the troposphere
// (exact solution of the hydrostatic equation for a constant temperature lapse rate).
// That model is valid up to 11 km, so it stays accurate over the 1 km the rocket flies.
// calibrateTo() anchors the scale on the ground, so getHeight_m() is the height above
// the calibration point (launch pad) instead of an absolute altitude.
//
// Velocity is estimated with a least squares fit (the slope) over the most recent height
// samples. For the same delay that filters the sensor noise a lot better than a plain
// difference or an exponential filter, which is what apogee detection needs. The fit
// uses the real sample timestamps, so it also survives an irregular update rate.
//
// Usage:
//     barometer.update();     // polls the sensor, roughly every loop
//     calculator.update();    // consumes that reading
//     ...
//     calculator.calibrateTo(0.0f); // once, while the rocket sits on the pad
class BarometricHeightCalculator {
public:
    explicit BarometricHeightCalculator(hardware::Barometer& barometer)
        : _barometer(barometer) {}

    float getHeight_m() const { return _height_m; }

    float getVelocity_m_s() const { return _velocity_m_s; }

    bool isCalibrated() const { return _isCalibrated; }

    // Sets the currently measured height to height_m, e.g. calibrateTo(0.0f) on the pad.
    // Does nothing while the barometer has not produced a valid measurement yet.
    void calibrateTo(float height_m);

    // Consumes the current barometer reading and updates height and velocity.
    // It does not talk to the sensor itself, so Barometer::update() has to run first.
    void update();

private:
    struct HeightSample {
        float height_m;
        uint32_t time_us;
    };

    // Troposphere model constants (International Standard Atmosphere).
    static constexpr float LAPSE_RATE_K_m = 0.0065f;
    static constexpr float BAROMETRIC_EXPONENT = 0.190263f; // R * L / g0, dimensionless
    static constexpr float CELSIUS_TO_KELVIN = 273.15f;
    static constexpr float ISA_SEA_LEVEL_PRESSURE_Pa = 101325.0f;
    static constexpr float ISA_SEA_LEVEL_TEMPERATURE_K = 288.15f;

    // Sliding window the velocity is fitted over. Lengthening it smooths the velocity
    // more but makes it follow the trajectory slower, shortening it does the opposite.
    // 200 ms keeps the sensor noise low while the delay stays far below what apogee
    // detection needs (see the notes in the .cpp).
    static constexpr uint32_t VELOCITY_WINDOW_us = 200000; // 200 ms
    static constexpr size_t VELOCITY_WINDOW_CAPACITY = 128;

    // A slope fitted through a nearly empty window is pure noise, so nothing is
    // published until the samples actually span a decent part of the window.
    static constexpr uint32_t VELOCITY_MIN_WINDOW_us = VELOCITY_WINDOW_us / 2;

    void pushHeightSample(float height_m, uint32_t time_us);
    void updateVelocity();

    float pressureToHeight_m(float pressure_Pa) const;

    hardware::Barometer& _barometer;

    float _height_m = 0.0f;
    float _velocity_m_s = 0.0f;

    bool _isCalibrated = false;

    // Height is measured relative to the calibration point of calibrateTo().
    float _referencePressure_Pa = ISA_SEA_LEVEL_PRESSURE_Pa;
    float _referenceTemperature_K = ISA_SEA_LEVEL_TEMPERATURE_K;
    float _referenceHeight_m = 0.0f;

    // Ring buffer with the most recent height samples used for the velocity fit.
    HeightSample _heightSamples[VELOCITY_WINDOW_CAPACITY] = {};
    size_t _oldestSampleIndex = 0;
    size_t _sampleCount = 0;

    // Last pressure that was processed, used to detect new barometer readings.
    float _lastPressure_Pa = 0.0f;
};