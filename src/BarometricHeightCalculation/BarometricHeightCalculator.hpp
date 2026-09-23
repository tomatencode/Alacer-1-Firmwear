#pragma once

#include <cstdint>

#include "../hardwareIO/barometer/Barometer.hpp"


// Turns the pressure measured by a barometer into a height.
//
// Height is calculated with the international barometric formula for the troposphere
// (exact solution of the hydrostatic equation for a constant temperature lapse rate).
// That model is valid up to 11 km and stays within ~15 m at 1 km for any realistic
// temperature profile, so it covers the 1 km the rocket flies.
//
// calibrateTo() anchors the scale on the ground, so getHeight_m() is the height above
// the calibration point (launch pad) instead of an absolute altitude. The height is
// recalculated from the latest measurement on every call, so it is always current and
// there is nothing to update in the main loop besides the barometer itself.
//
// Usage:
//     barometer.update();            // polls the sensor, roughly every loop
//     float h = calculator.getHeight_m();
//     calculator.calibrateTo(0.0f);  // once, while the rocket sits on the pad
//
// Smoothing and the vertical velocity live in VerticalMovementTracker, which samples
// this class.
class BarometricHeightCalculator {
public:
    explicit BarometricHeightCalculator(hardware::Barometer& barometer)
        : _barometer(barometer) {}

    // Height above the calibration point, calculated from the current measurement.
    float getHeight_m() const;

    bool isCalibrated() const { return _isCalibrated; }

    // Sets the currently measured height to height_m, e.g. calibrateTo(0.0f) on the pad.
    // Does nothing while the barometer has not produced a valid measurement yet.
    void calibrateTo(float height_m);

private:
    // Troposphere model constants (International Standard Atmosphere).
    static constexpr float LAPSE_RATE_K_m = 0.0065f;
    static constexpr float BAROMETRIC_EXPONENT = 0.190263f; // R * L / g0, dimensionless
    static constexpr float CELSIUS_TO_KELVIN = 273.15f;
    static constexpr float ISA_SEA_LEVEL_PRESSURE_Pa = 101325.0f;
    static constexpr float ISA_SEA_LEVEL_TEMPERATURE_K = 288.15f;

    float pressureToHeight_m(float pressure_Pa) const;

    hardware::Barometer& _barometer;

    bool _isCalibrated = false;

    // Height is measured relative to the calibration point of calibrateTo().
    float _referencePressure_Pa = ISA_SEA_LEVEL_PRESSURE_Pa;
    float _referenceTemperature_K = ISA_SEA_LEVEL_TEMPERATURE_K;
    float _referenceHeight_m = 0.0f;
};