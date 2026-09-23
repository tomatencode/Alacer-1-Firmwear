#include "BarometricHeightCalculator.hpp"

#include <Arduino.h>
#include <cmath>

void BarometricHeightCalculator::calibrateTo(float height_m) {
    const float pressure_Pa = _barometer.getPressure_Pa();

    if (pressure_Pa <= 0.0f) { return; } // no valid measurement yet, keep the old reference

    _referencePressure_Pa = pressure_Pa;
    // The temperature at the reference point sets the height scale of the atmosphere
    // below it, it is not the temperature of the air the rocket is currently flying in.
    _referenceTemperature_K = _barometer.getTemperature_C() + CELSIUS_TO_KELVIN;
    _referenceHeight_m = height_m;

    _height_m = height_m;

    // The samples in the window stay valid, a new reference only shifts all of them by
    // the same offset, which the velocity fit is insensitive to.
    _isCalibrated = true;
}

void BarometricHeightCalculator::update() {
    const float pressure_Pa = _barometer.getPressure_Pa();

    if (pressure_Pa <= 0.0f) { return; } // sensor not ready or read failed

    // The barometer is polled by the main loop, only new readings are worth processing.
    if (pressure_Pa == _lastPressure_Pa) { return; }
    _lastPressure_Pa = pressure_Pa;

    _height_m = _referenceHeight_m + pressureToHeight_m(pressure_Pa);

    pushHeightSample(_height_m, micros());
    updateVelocity();
}

float BarometricHeightCalculator::pressureToHeight_m(float pressure_Pa) const {
    // Solves dP / dh = -g0 * P / (R * T) for a linear temperature profile T = T_ref - L * h:
    //   h = (T_ref / L) * (1 - (P / P_ref)^(R * L / g0))
    // which is the same scale the International Standard Atmosphere uses down here.
    const float pressureRatio = pressure_Pa / _referencePressure_Pa;

    return (_referenceTemperature_K / LAPSE_RATE_K_m) * (1.0f - powf(pressureRatio, BAROMETRIC_EXPONENT));
}

void BarometricHeightCalculator::pushHeightSample(float height_m, uint32_t time_us) {
    if (_sampleCount == VELOCITY_WINDOW_CAPACITY) {
        _oldestSampleIndex = (_oldestSampleIndex + 1) % VELOCITY_WINDOW_CAPACITY;
        --_sampleCount;
    }

    const size_t writeIndex = (_oldestSampleIndex + _sampleCount) % VELOCITY_WINDOW_CAPACITY;
    _heightSamples[writeIndex] = HeightSample{height_m, time_us};
    ++_sampleCount;

    // Drop the samples that fell out of the time window, but always keep two to fit.
    while (_sampleCount > 2 &&
           (time_us - _heightSamples[_oldestSampleIndex].time_us) > VELOCITY_WINDOW_us) {
        _oldestSampleIndex = (_oldestSampleIndex + 1) % VELOCITY_WINDOW_CAPACITY;
        --_sampleCount;
    }
}

void BarometricHeightCalculator::updateVelocity() {
    if (_sampleCount < 2) { // nothing to fit yet
        _velocity_m_s = 0.0f;
        return;
    }

    const size_t newestIndex = (_oldestSampleIndex + _sampleCount - 1) % VELOCITY_WINDOW_CAPACITY;
    const uint32_t newestTime_us = _heightSamples[newestIndex].time_us;

    if ((newestTime_us - _heightSamples[_oldestSampleIndex].time_us) < VELOCITY_MIN_WINDOW_us) {
        _velocity_m_s = 0.0f;
        return;
    }

    // Least squares fit of height = offset + velocity * t over the window, with t measured
    // backwards from the newest sample so the fitted slope is the vertical velocity.
    float sumT = 0.0f;
    float sumHeight = 0.0f;
    float sumTT = 0.0f;
    float sumTHeight = 0.0f;

    for (size_t i = 0; i < _sampleCount; ++i) {
        const HeightSample& sample = _heightSamples[(_oldestSampleIndex + i) % VELOCITY_WINDOW_CAPACITY];

        const float t_s = -static_cast<float>(newestTime_us - sample.time_us) * 1e-6f;

        sumT += t_s;
        sumHeight += sample.height_m;
        sumTT += t_s * t_s;
        sumTHeight += t_s * sample.height_m;
    }

    const float sampleCount = static_cast<float>(_sampleCount);
    const float denominator = sampleCount * sumTT - sumT * sumT;

    if (denominator <= 0.0f) { return; } // all samples share one timestamp

    _velocity_m_s = (sampleCount * sumTHeight - sumT * sumHeight) / denominator;
}

