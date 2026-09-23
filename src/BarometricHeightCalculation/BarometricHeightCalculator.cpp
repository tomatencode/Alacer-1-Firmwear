#include "BarometricHeightCalculator.hpp"

#include <cmath>

void BarometricHeightCalculator::calibrateTo(float height_m) {
    const float pressure_Pa = _barometer.getPressure_Pa();

    if (pressure_Pa <= 0.0f) { return; } // no valid measurement yet, keep the old reference

    _referencePressure_Pa = pressure_Pa;
    // The temperature at the reference point sets the height scale of the atmosphere
    // below it, it is not the temperature of the air the rocket is currently flying in.
    _referenceTemperature_K = _barometer.getTemperature_C() + CELSIUS_TO_KELVIN;
    _referenceHeight_m = height_m;

    _isCalibrated = true;
}

float BarometricHeightCalculator::getHeight_m() const {
    const float pressure_Pa = _barometer.getPressure_Pa();

    if (pressure_Pa <= 0.0f) { return _referenceHeight_m; } // no valid measurement yet

    return _referenceHeight_m + pressureToHeight_m(pressure_Pa);
}

float BarometricHeightCalculator::pressureToHeight_m(float pressure_Pa) const {
    // Solves dP / dh = -g0 * P / (R * T) for a linear temperature profile T = T_ref - L * h:
    //   h = (T_ref / L) * (1 - (P / P_ref)^(R * L / g0))
    // which is the same scale the International Standard Atmosphere uses down here.
    const float pressureRatio = pressure_Pa / _referencePressure_Pa;

    return (_referenceTemperature_K / LAPSE_RATE_K_m) * (1.0f - powf(pressureRatio, BAROMETRIC_EXPONENT));
}

