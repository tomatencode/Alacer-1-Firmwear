#include "PositionAccumulator.hpp"

PositionAccumulator::PositionAccumulator(hardware::IMU& imu, IMURocketCoordinateConverter& imuRocketConverter, RotationAccumulator& rotationAccumulator, uint32_t maxDt_us)
    : _imu(imu),
      _imuRocketConverter(imuRocketConverter),
      _rotationAccumulator(rotationAccumulator),
      _maxDt_us(maxDt_us),
      _position_m(Eigen::Vector3f::Zero()),
      _velocity_m_s(Eigen::Vector3f::Zero())
{}

void PositionAccumulator::setPosition_m_Velocity_m_s(const Eigen::Vector3f& position, const Eigen::Vector3f& velocity) {
    _lastUpdateTime_us = micros(); // start accumulating from the current time
    _position_m = position;
    _velocity_m_s = velocity;
}

Eigen::Vector3f PositionAccumulator::getPosition_m() const {
    return _position_m;
}

Eigen::Vector3f PositionAccumulator::getVelocity_m_s() const {
    return _velocity_m_s;
}

void PositionAccumulator::holdPosition() {
    auto deltaTimeOpt = tryGetDeltaTime_s();
    if (!deltaTimeOpt.has_value()) {
        return;
    }
    float deltaTime_s = deltaTimeOpt.value();


    Eigen::Vector3f acceleration_world_m_s2 = getAccelerationWorld_m_s2();

    _velocity_m_s += acceleration_world_m_s2 * deltaTime_s;
}

void PositionAccumulator::updatePosition() {
    auto deltaTimeOpt = tryGetDeltaTime_s();
    if (!deltaTimeOpt.has_value()) {
        return;
    }
    float deltaTime_s = deltaTimeOpt.value();

    Eigen::Vector3f acceleration_world_m_s2 = getAccelerationWorld_m_s2();

    _velocity_m_s += acceleration_world_m_s2 * deltaTime_s;

    _position_m += _velocity_m_s * deltaTime_s;
}

std::optional<float> PositionAccumulator::tryGetDeltaTime_s() {
    uint32_t currentTime_us = micros();

    if (_lastUpdateTime_us == 0) { // First update, initialize the last update time
        _lastUpdateTime_us = currentTime_us;
        return std::nullopt;
    }

    uint32_t deltaTime_us = currentTime_us - _lastUpdateTime_us;
    float deltaTime_s = min(deltaTime_us, _maxDt_us) * 1e-6f;
    _lastUpdateTime_us = currentTime_us;
    return deltaTime_s;
}

Eigen::Vector3f PositionAccumulator::getAccelerationWorld_m_s2() const {
    auto accel_data = _imu.getAccel();

    Eigen::Vector3f acceleration_imu_m_s2 = {
        accel_data.x_m_s2,
        accel_data.y_m_s2,
        accel_data.z_m_s2
    };

    Eigen::Vector3f acceleration_rocket_m_s2 = _imuRocketConverter.rotateIMUVectorToRocket(acceleration_imu_m_s2);
    Eigen::Vector3f acceleration_world_m_s2 = _rotationAccumulator.getRotationQuaternion() * acceleration_rocket_m_s2;

    acceleration_world_m_s2 -= Eigen::Vector3f{0.0f, 0.0f, 9.81f};

    return acceleration_world_m_s2;
}