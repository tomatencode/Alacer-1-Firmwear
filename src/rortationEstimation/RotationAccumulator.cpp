#include "RotationAccumulator.hpp"

RotationAccumulator::RotationAccumulator(hardware::IMU& imu, IMURocketCoordinateConverter& imuRocketConverter, uint32_t maxDt_us)
    : _imu(imu), _imuRocketConverter(imuRocketConverter), _maxDt_us(maxDt_us)
{}

Eigen::Quaternionf RotationAccumulator::getRotationQuaternion() const {
    return _rotation;
}

Eigen::Matrix3f RotationAccumulator::getRotationMatrix() const {
    return _rotation.toRotationMatrix();
}

Eigen::Vector3f RotationAccumulator::getEulerAngles_rad() const {
    return _rotation.toRotationMatrix().eulerAngles(0, 1, 2);
}

void RotationAccumulator::setRotationQuaternion(const Eigen::Quaternionf& rotation) {
    _lastUpdate_us = micros(); // accumulation shuld start here
    _rotation = rotation.normalized();
}

void RotationAccumulator::setRotationMatrix(const Eigen::Matrix3f& rotationMatrix) {
    _lastUpdate_us = micros(); // accumulation shuld start here
    _rotation = Eigen::Quaternionf(rotationMatrix).normalized();
}

void RotationAccumulator::setEulerAngles_rad(const Eigen::Vector3f& eulerAngles) {
    _lastUpdate_us = micros(); // accumulation shuld start here
    _rotation = Eigen::Quaternionf(
        Eigen::AngleAxisf(eulerAngles[0], Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(eulerAngles[1], Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(eulerAngles[2], Eigen::Vector3f::UnitZ())
    ).normalized();
}

Eigen::Vector3f RotationAccumulator::getAngularVelocity_rad_s() const {
    return _angularVelocity_rad_s;
}

void RotationAccumulator::holdOrientation() {
    const auto gyro = _imu.getGyro();
    Eigen::Vector3f angularVelocityIMU_rad_s = {
        gyro.x_rad_s,
        gyro.y_rad_s,
        gyro.z_rad_s
    };

    _angularVelocity_rad_s = _imuRocketConverter.rotateIMUVectorToRocket(angularVelocityIMU_rad_s);

    _lastUpdate_us = micros();
}

void RotationAccumulator::update() {
    const uint32_t currentTime_us = micros();

    auto gyro = _imu.getGyro();
    
    Eigen::Vector3f angularVelocityIMU_rad_s = {
        gyro.x_rad_s,
        gyro.y_rad_s,
        gyro.z_rad_s    
    };

    _angularVelocity_rad_s = _imuRocketConverter.rotateIMUVectorToRocket(angularVelocityIMU_rad_s);


    if (_lastUpdate_us == 0) { // First update, initialize the last update time
        _lastUpdate_us = currentTime_us;
        return;
    }

    const uint32_t measuredDt_us = currentTime_us - _lastUpdate_us;
    _lastUpdate_us = currentTime_us;

    const float dt = min(measuredDt_us, _maxDt_us) * 1e-6f;

    const Eigen::Vector3f deltaTheta = _angularVelocity_rad_s * dt;
    const float angle = deltaTheta.norm();

    if (angle > 1e-6f) {
        const Eigen::Quaternionf deltaRotation(Eigen::AngleAxisf(angle, deltaTheta / angle));
        _rotation = (_rotation * deltaRotation).normalized();
    }
}