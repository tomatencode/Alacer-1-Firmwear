#pragma once

#include <ArduinoEigen.h>

#include "IMURocketCoordinateConverter.hpp"

#include "../hardware/imu/IMU.hpp"

class RotationAccumulator {
public:
    RotationAccumulator(hardware::IMU& imu, IMURocketCoordinateConverter& imuRocketConverter, uint32_t maxDt_us);

    Eigen::Quaternionf getRotationQuaternion() const;
    Eigen::Matrix3f getRotationMatrix() const;
    Eigen::Vector3f getEulerAngles_rad() const;

    void setRotationQuaternion(const Eigen::Quaternionf& rotation);
    void setRotationMatrix(const Eigen::Matrix3f& rotationMatrix);
    void setEulerAngles_rad(const Eigen::Vector3f& eulerAngles);

    Eigen::Vector3f getAngularVelocity_rad_s() const;

    void holdOrientation();
    void updateOrientation();

private:
    hardware::IMU& _imu;

    uint32_t _lastUpdate_us = 0;

    uint32_t _maxDt_us;

    Eigen::Quaternionf _rotation = Eigen::Quaternionf::Identity();
    Eigen::Vector3f _angularVelocity_rad_s = Eigen::Vector3f::Zero();

    IMURocketCoordinateConverter& _imuRocketConverter;
};