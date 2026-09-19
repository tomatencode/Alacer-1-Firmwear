#include "Gimbal.hpp"

namespace hardware {

Gimbal::Gimbal(
    hardware::Servo& pitchServo, hardware::Servo& yawServo,
    GimbalPos initialTarget,
    float minPitch_deg, float maxPitch_deg,
    float minYaw_deg, float maxYaw_deg,
    float pitchServoTranslation, float yawServoTranslation,
    float pitchServoTrim_deg, float yawServoTrim_deg
) :
    _pitchServo(pitchServo),
    _yawServo(yawServo),
    _target(initialTarget),
    _minPitch_deg(minPitch_deg),
    _maxPitch_deg(maxPitch_deg),
    _minYaw_deg(minYaw_deg),
    _maxYaw_deg(maxYaw_deg),
    _pitchServoTrim_deg(pitchServoTrim_deg),
    _yawServoTrim_deg(yawServoTrim_deg),
    _pitchServoTranslation(pitchServoTranslation),
    _yawServoTranslation(yawServoTranslation)
{}

void Gimbal::begin() {
    setTarget(_target);
}

void Gimbal::setTarget(const GimbalPos& target) {
    float clampedPitch = std::clamp(target.pitch_deg, _minPitch_deg, _maxPitch_deg);
    float clampedYaw = std::clamp(target.yaw_deg, _minYaw_deg, _maxYaw_deg);

    float pitchServoAngle = clampedPitch/_pitchServoTranslation + _pitchServoTrim_deg;
    float yawServoAngle = clampedYaw/_yawServoTranslation + _yawServoTrim_deg;

    _pitchServo.setTarget(pitchServoAngle);
    _yawServo.setTarget(yawServoAngle);

    _target = {clampedPitch, clampedYaw};
}

Gimbal::GimbalPos Gimbal::getTarget() const {
    return _target;
}

Gimbal::GimbalPos Gimbal::getCurrentPos() const {
    float currentPitch = (_pitchServo.getPosition_deg() - _pitchServoTrim_deg) * _pitchServoTranslation;
    float currentYaw = (_yawServo.getPosition_deg() - _yawServoTrim_deg) * _yawServoTranslation;

    return {currentPitch, currentYaw};
}

} // namespace hardware