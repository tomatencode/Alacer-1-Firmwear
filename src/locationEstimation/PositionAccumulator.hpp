#pragma once

#include <cstdint>
#include <optional>

#include "../hardware/imu/IMU.hpp"
#include "./RotationAccumulator.hpp"

#include <ArduinoEigen.h>

class PositionAccumulator {
public:
    PositionAccumulator(hardware::IMU& imu, IMURocketCoordinateConverter& imuRocketConverter, RotationAccumulator& rotationAccumulator,  uint32_t maxDt_us);

    Eigen::Vector3f getPosition_m() const;
    Eigen::Vector3f getVelocity_m_s() const;

    void setPosition_m_Velocity_m_s(const Eigen::Vector3f& position, const Eigen::Vector3f& velocity);

    void holdPosition();
    void updatePosition();

private:
    hardware::IMU& _imu;
    IMURocketCoordinateConverter& _imuRocketConverter;
    RotationAccumulator& _rotationAccumulator;

    uint32_t _lastUpdateTime_us = 0;
    uint32_t _maxDt_us;

    Eigen::Vector3f _position_m;
    Eigen::Vector3f _velocity_m_s;
    
    std::optional<float> tryGetDeltaTime_s();
    Eigen::Vector3f getAccelerationWorld_m_s2() const;
};