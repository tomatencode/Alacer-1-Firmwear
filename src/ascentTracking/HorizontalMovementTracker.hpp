#pragma once

#include <stdint.h>
#include <cstddef>
#include <optional>
#include <Arduino.h>
#include <ArduinoEigen.h>

#include "./VerticalMovementTracker.hpp"
#include "../rotationEstimation/RotationAccumulator.hpp"


// tracks horizontal movement on ascent by integrating horizontal displacement over time
// horizontal displacement is calculated based on rotation and vertical movement
class HorizontalMovementTracker {
public:

    enum class AscentStage {
        ASCENDING,
        COASTING,
        DESCENDING_CHUTE
    };

    HorizontalMovementTracker(VerticalMovementTracker& verticalTracker, RotationAccumulator& rotationAccumulator);

    void stopTracking();
    void startTracking();
    void reset();

    Eigen::Vector2f getTotalMovement_m() const;

    Eigen::Vector2f getVelocity_m_s() const { return _velocity_m_s; }
    bool hasVelocityEstimate() const { return _hasVelocityEstimate; }

    void setAscentStage(AscentStage stage);
    AscentStage getAscendStage() const { return _currentAscentStage; }

    void update();
private:
    // One horizontal position sample, so velocity can be fitted from a window of
    // samples the same way the vertical tracker fits height samples.
    struct MovementSample {
        Eigen::Vector2f position_m;
        uint32_t time_us;
    };

    static constexpr uint32_t VELOCITY_WINDOW_us = 200000; // 200 ms
    static constexpr size_t VELOCITY_WINDOW_CAPACITY = 128;

    static constexpr uint32_t VELOCITY_MIN_WINDOW_us = VELOCITY_WINDOW_us / 2;

    // Below this the nose axis is too close to horizontal for the
    // deltaHeight / noseUp model to say anything useful.
    static constexpr float MIN_NOSE_UP = 1e-3f;

    void pushMovementSample(const Eigen::Vector2f& position_m, uint32_t time_us);
    void updateVelocityEstimate();

    Eigen::Vector2f getAngleBasedMovementDelta();

    VerticalMovementTracker& verticalTracker;
    RotationAccumulator& rotationAccumulator;

    bool _tracking = false;

    uint32_t _lastUpdateTime_us = 0;

    AscentStage _currentAscentStage = AscentStage::ASCENDING;

    std::optional<float> _lastHeight_m;

    Eigen::Vector2f _totalMovement;
    Eigen::Vector2f _velocity_m_s;
    bool _hasVelocityEstimate = false;

    MovementSample _movementSamples[VELOCITY_WINDOW_CAPACITY] = {};
    size_t _oldestSampleIndex = 0;
    size_t _sampleCount = 0;
};