#pragma once

#include <cstddef>
#include <cstdint>

#include "../BarometricHeightCalculation/BarometricHeightCalculator.hpp"


class VerticalMovementTracker {
public:
    explicit VerticalMovementTracker(BarometricHeightCalculator& heightSource)
        : _heightSource(heightSource) {}

    void reset();

    float getHeight_m() const { return _height_m; }

    float getVelocity_m_s() const { return _velocity_m_s; }
    bool hasVelocityEstimate() const { return _hasVelocityEstimate; }

    float getMaxHeight_m() const { return _maxHeight_m; }
    void update();

private:
    struct HeightSample {
        float height_m;
        uint32_t time_us;
    };

    static constexpr uint32_t VELOCITY_WINDOW_us = 200000; // 200 ms
    static constexpr size_t VELOCITY_WINDOW_CAPACITY = 128;

    static constexpr uint32_t VELOCITY_MIN_WINDOW_us = VELOCITY_WINDOW_us / 2;

    void pushHeightSample(float height_m, uint32_t time_us);
    void updateEstimates();

    BarometricHeightCalculator& _heightSource;

    float _height_m = 0.0f;
    float _velocity_m_s = 0.0f;
    float _maxHeight_m = 0.0f;
    bool _hasVelocityEstimate = false;

    float _lastSampledHeight_m = 0.0f;
    bool _hasSampled = false;

    HeightSample _heightSamples[VELOCITY_WINDOW_CAPACITY] = {};
    size_t _oldestSampleIndex = 0;
    size_t _sampleCount = 0;
};