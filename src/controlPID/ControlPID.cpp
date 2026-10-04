#include "ControlPID.hpp"

#include <Arduino.h>
#include <cmath>


ControlPID::ControlPID(RotationAccumulator& rotationAccumulator, hardware::Gimbal& gimbal)
    : _rotationAccumulator(rotationAccumulator),
      _gimbal(gimbal),
      _targetAngle(std::nullopt),
      _isControlling(false),
      _kp(std::nullopt),
      _ki(std::nullopt),
      _kd(std::nullopt) {
}

bool ControlPID::canStartControlling() const {
    return _kp.has_value() && _ki.has_value() && _kd.has_value() && _targetAngle.has_value();
}

void ControlPID::setPIDParameters(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

std::optional<ControlPID::PIDParameters> ControlPID::getPIDParameters() const {
    if (!_kp.has_value() || !_ki.has_value() || !_kd.has_value()) {
        return std::nullopt;
    }
    return PIDParameters{_kp.value(), _ki.value(), _kd.value()};
}

void ControlPID::setTarget(const Eigen::Quaternionf& targetAngle) {
    _targetAngle = targetAngle;
}

std::optional<Eigen::Quaternionf> ControlPID::getTarget() const {
    return _targetAngle;
}

void ControlPID::startControlling() {
  if (!canStartControlling()) return;
  _isControlling = true;
}
void ControlPID::stopControlling() {
  _isControlling = false;
}

bool ControlPID::setControlling(bool controlling) {
  if (controlling) {
    startControlling();
  } else {
    stopControlling();
  }
  return _isControlling == controlling;
}

void ControlPID::clearConfig() {
    _targetAngle = std::nullopt;
    _isControlling = false;
    _kp = std::nullopt;
    _ki = std::nullopt;
    _kd = std::nullopt;
    _integralPitch_rad = 0.0f;
    _integralYaw_rad = 0.0f;
}

void ControlPID::update() {
    if (!_isControlling || !canStartControlling()) {
        return;
    }

    if (_lastUpdateTime_us == 0) { // first update, initialize the last update time
        _lastUpdateTime_us = micros();
        return;
    }

    uint32_t currentTime_us = micros();
    uint32_t dt_us = min(currentTime_us - _lastUpdateTime_us, MAX_DT_us);
    float deltaTime_s = dt_us * 1e-6f;
    _lastUpdateTime_us = currentTime_us;

    // Quaternion attitude error, expressed in the body/rocket frame.
    // qCurrent maps body -> world, qTarget is the desired body -> world.
    // qErr = qCurrent^-1 * qTarget is the rotation that takes the current
    // body frame to the target frame. Its rotation-vector (angle * axis)
    // gives a singularity-free, wrap-free error. X = pitch, Y = yaw,
    // Z (roll) is intentionally ignored: axisymmetric rocket, no roll control.
    Eigen::Quaternionf qCurrent = _rotationAccumulator.getRotationQuaternion().normalized();
    Eigen::Quaternionf qTarget = _targetAngle->normalized();
    Eigen::Quaternionf qErr = qCurrent.conjugate() * qTarget;
    if (qErr.w() < 0.0f) {
        // Double-cover: q and -q are the same rotation, pick shortest path.
        qErr.coeffs() = -qErr.coeffs();
    }

    float pitchError_rad = 0.0f;
    float yawError_rad = 0.0f;
    {
        float w = qErr.w();
        if (w > 1.0f) w = 1.0f;
        if (w < -1.0f) w = -1.0f;
        const float angle_rad = 2.0f * acosf(w);
        const float sinHalfAngle = sqrtf(1.0f - w * w);
        if (sinHalfAngle > 1e-6f) {
            const float invSin = 1.0f / sinHalfAngle;
            Eigen::Vector3f axis(qErr.x() * invSin, qErr.y() * invSin, qErr.z() * invSin);
            Eigen::Vector3f errRotVec = angle_rad * axis;
            pitchError_rad = errRotVec.x();
            yawError_rad = errRotVec.y();
            // errRotVec.z() is roll error: measured but not controlled.
        }
        // else: angle ~0 or ~2pi, error stays zero.
    }

    _integralPitch_rad += pitchError_rad * deltaTime_s;
    _integralYaw_rad += yawError_rad * deltaTime_s;

    Eigen::Vector3f angularVelocity_rad_s = _rotationAccumulator.getAngularVelocity_rad_s();
    float pitchRate_rad_s = angularVelocity_rad_s[0];
    float yawRate_rad_s = angularVelocity_rad_s[1];

    float pitchOutput_rad = _kp.value() * pitchError_rad
                          + _ki.value() * _integralPitch_rad
                          - _kd.value() * pitchRate_rad_s;

    float yawOutput_rad = _kp.value() * yawError_rad
                        + _ki.value() * _integralYaw_rad
                        - _kd.value() * yawRate_rad_s;

    hardware::Gimbal::GimbalPos targetPos{
        .pitch_deg = pitchOutput_rad * 180.0f / std::numbers::pi_v<float>,
        .yaw_deg = yawOutput_rad * 180.0f / std::numbers::pi_v<float>
    };

    _gimbal.setTarget(targetPos);
}