#include "VerticalMovementTracker.hpp"

#include <Arduino.h>

void VerticalMovementTracker::update() {
    const float height_m = _heightSource.getHeight_m();

    // The main loop runs far more often than the barometer produces fresh readings, a
    // height that did not change carries no new information about the movement.
    if (_hasSampled && height_m == _lastSampledHeight_m) { return; }
    _hasSampled = true;
    _lastSampledHeight_m = height_m;

    pushHeightSample(height_m, micros());
    updateEstimates();
}

void VerticalMovementTracker::reset() {
    _height_m = 0.0f;
    _velocity_m_s = 0.0f;
    _maxHeight_m = 0.0f;
    _hasVelocityEstimate = false;
    _hasSampled = false;
    _oldestSampleIndex = 0;
    _sampleCount = 0;
}

void VerticalMovementTracker::pushHeightSample(float height_m, uint32_t time_us) {
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

void VerticalMovementTracker::updateEstimates() {
    const size_t newestIndex = (_oldestSampleIndex + _sampleCount - 1) % VELOCITY_WINDOW_CAPACITY;
    const uint32_t newestTime_us = _heightSamples[newestIndex].time_us;
    const uint32_t windowSpan_us = newestTime_us - _heightSamples[_oldestSampleIndex].time_us;

    // While the fit means nothing yet the newest reading is the best guess we have.
    _height_m = _heightSamples[newestIndex].height_m;
    _velocity_m_s = 0.0f;
    _hasVelocityEstimate = false;

    if (_sampleCount >= 2 && windowSpan_us >= VELOCITY_MIN_WINDOW_us) {
        // Least squares fit of height = offset + velocity * t over the window, with t
        // measured backwards from the newest sample, so the fitted slope is the vertical
        // velocity and the fitted offset is the height at the newest sample.
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

        if (denominator > 0.0f) { // zero when every sample shares one timestamp
            _velocity_m_s = (sampleCount * sumTHeight - sumT * sumHeight) / denominator;
            _height_m = (sumHeight - _velocity_m_s * sumT) / sampleCount;
            _hasVelocityEstimate = true;
        }
    }

    if (_height_m > _maxHeight_m) { _maxHeight_m = _height_m; }
}
