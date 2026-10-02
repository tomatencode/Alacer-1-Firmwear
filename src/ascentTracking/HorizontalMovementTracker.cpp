#include "HorizontalMovementTracker.hpp"

#include <Arduino.h>
#include <cmath>


HorizontalMovementTracker::HorizontalMovementTracker(VerticalMovementTracker& verticalTracker, RotationAccumulator& rotationAccumulator)
    : verticalTracker(verticalTracker),
      rotationAccumulator(rotationAccumulator),
      _totalMovement(Eigen::Vector2f::Zero()),
      _velocity_m_s(Eigen::Vector2f::Zero())
{
}

Eigen::Vector2f HorizontalMovementTracker::getTotalMovement_m() const {
    return _totalMovement;
}

void HorizontalMovementTracker::stopTracking() {
    _tracking = false;
}

void HorizontalMovementTracker::startTracking() {
    _tracking = true;
    _lastUpdateTime_us = 0; // // dont integrate the paused interval
}

void HorizontalMovementTracker::reset() {
    _tracking = false;
    _totalMovement = Eigen::Vector2f::Zero();
    _velocity_m_s = Eigen::Vector2f::Zero();
    _hasVelocityEstimate = false;
    _lastHeight_m = std::nullopt;
    _oldestSampleIndex = 0;
    _sampleCount = 0;
    // Stage and clock must restart too, otherwise a second flight would begin in
    // DESCENDING_CHUTE and integrate a zero velocity forever, recording no movement.
    _currentAscentStage = AscentStage::ASCENDING;
    _lastUpdateTime_us = 0;
}

void HorizontalMovementTracker::setAscentStage(AscentStage stage) {
    _currentAscentStage = stage;
}

void HorizontalMovementTracker::update() {
    if (!_tracking) { return; }

    const uint32_t currentTime_us = micros();
    
    if (_lastUpdateTime_us == 0) {
        _lastUpdateTime_us = currentTime_us;
        return;
    }
    const float deltaTime_s = (currentTime_us - _lastUpdateTime_us) * 1e-6f;
    _lastUpdateTime_us = currentTime_us;

    Eigen::Vector2f horizontalDisplacement = Eigen::Vector2f::Zero();

    switch (_currentAscentStage)
    {
    case AscentStage::ASCENDING:
        horizontalDisplacement = getAngleBasedMovementDelta();
        break;
    case AscentStage::COASTING:
        horizontalDisplacement = getVelocity_m_s() * deltaTime_s;
        break;
    case AscentStage::DESCENDING_CHUTE:
        horizontalDisplacement = getVelocity_m_s() * deltaTime_s / 2.0f; // slower horizontal movement during descent because of chute
        break;
    }


    _totalMovement += horizontalDisplacement;

    pushMovementSample(_totalMovement, currentTime_us);
    updateVelocityEstimate();
}

Eigen::Vector2f HorizontalMovementTracker::getAngleBasedMovementDelta() {
    const float currentHeight_m = verticalTracker.getHeight_m();
    // First reading only anchors the height, there is no displacement to account for yet.
    if (!_lastHeight_m.has_value()) {
        _lastHeight_m = currentHeight_m;
        return Eigen::Vector2f::Zero();
    }

    const float deltaHeight_m = currentHeight_m - _lastHeight_m.value();
    _lastHeight_m = currentHeight_m;

    // A height that did not change carries no new information about the movement, exactly
    // like the vertical tracker: this model only produces displacement from a height
    // change, so there is nothing to integrate and nothing new to fit.
    if (deltaHeight_m == 0.0f) { return Eigen::Vector2f::Zero(); }

    const Eigen::Quaternionf currentRotation = rotationAccumulator.getRotationQuaternion();

    // Nose axis (+body Z) in world coordinates. World +Z is up because the accumulator
    // starts at Identity on the pad and roll is about Z (see ControlPID.cpp).
    const float noseUp = (currentRotation * Eigen::Vector3f::UnitZ()).z();

    Eigen::Vector2f horizontalDisplacement = Eigen::Vector2f::Zero();
    if (std::abs(noseUp) > MIN_NOSE_UP) {   // rocket near-horizontal: model breaks down
        // deltaHeight is only the vertical component of the along-axis travel, so the
        // along-axis distance is deltaHeight / noseUp.
        horizontalDisplacement =
            (currentRotation * Eigen::Vector3f(0.0f, 0.0f, deltaHeight_m / noseUp)).head<2>();
    }
    
    return horizontalDisplacement;
}

void HorizontalMovementTracker::pushMovementSample(const Eigen::Vector2f& position_m, uint32_t time_us) {
    if (_sampleCount == VELOCITY_WINDOW_CAPACITY) {
        _oldestSampleIndex = (_oldestSampleIndex + 1) % VELOCITY_WINDOW_CAPACITY;
        --_sampleCount;
    }

    const size_t writeIndex = (_oldestSampleIndex + _sampleCount) % VELOCITY_WINDOW_CAPACITY;
    _movementSamples[writeIndex] = MovementSample{position_m, time_us};
    ++_sampleCount;

    // Drop the samples that fell out of the time window, but always keep two to fit.
    while (_sampleCount > 2 &&
           (time_us - _movementSamples[_oldestSampleIndex].time_us) > VELOCITY_WINDOW_us) {
        _oldestSampleIndex = (_oldestSampleIndex + 1) % VELOCITY_WINDOW_CAPACITY;
        --_sampleCount;
    }
}

void HorizontalMovementTracker::updateVelocityEstimate() {
    _velocity_m_s = Eigen::Vector2f::Zero();
    _hasVelocityEstimate = false;

    if (_sampleCount < 2) { return; }

    const size_t newestIndex = (_oldestSampleIndex + _sampleCount - 1) % VELOCITY_WINDOW_CAPACITY;
    const uint32_t newestTime_us = _movementSamples[newestIndex].time_us;
    const uint32_t windowSpan_us = newestTime_us - _movementSamples[_oldestSampleIndex].time_us;

    if (windowSpan_us < VELOCITY_MIN_WINDOW_us) { return; }

    // Least squares fit of position = offset + velocity * t over the window, with t
    // measured backwards from the newest sample, so the fitted slope is the horizontal
    // velocity. The normal equations are the same as the vertical tracker's scalar fit,
    // only sumTHeight becomes a Vector2f, so both axes fall out of one solve.
    float sumT = 0.0f;
    float sumTT = 0.0f;
    Eigen::Vector2f sumPosition = Eigen::Vector2f::Zero();
    Eigen::Vector2f sumTPosition = Eigen::Vector2f::Zero();

    for (size_t i = 0; i < _sampleCount; ++i) {
        const MovementSample& sample = _movementSamples[(_oldestSampleIndex + i) % VELOCITY_WINDOW_CAPACITY];

        const float t_s = -static_cast<float>(newestTime_us - sample.time_us) * 1e-6f;

        sumT += t_s;
        sumTT += t_s * t_s;
        sumPosition += sample.position_m;
        sumTPosition += t_s * sample.position_m;
    }

    const float sampleCount = static_cast<float>(_sampleCount);
    const float denominator = sampleCount * sumTT - sumT * sumT;

    if (denominator > 0.0f) { // zero when every sample shares one timestamp
        _velocity_m_s = (sampleCount * sumTPosition - sumT * sumPosition) / denominator;
        _hasVelocityEstimate = true;
    }
}