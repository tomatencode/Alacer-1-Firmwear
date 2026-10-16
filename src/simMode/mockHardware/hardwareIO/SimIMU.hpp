#pragma once
#include <cstdint>
#include <cstddef>

#include "simMode/SimRocket.hpp"
#include "hardwareIO/imu/IMU.hpp"
#include "helpers/coordinates/IMURocketCoordinateConverter.hpp"

namespace simHardware {

class SimIMU : public hardware::IMU {
public:
    explicit SimIMU(SimRocket &simRocket, IMURocketCoordinateConverter& imuRocketCoordinateConverter);

    void begin() override {};
    void update() override {};

    hardware::IMU::Accel getAccel() const override {
        Eigen::Vector3f accel_body = _simRocket.getAccelerationBody();
        Eigen::Vector3f accel = _imuRocketCoordinateConverter.rotateRocketVectorToIMU(accel_body);
        return hardware::IMU::Accel{accel.x(), accel.y(), accel.z()};
    };
    hardware::IMU::Gyro getGyro() const override {
        Eigen::Vector3f gyro_body = _simRocket.getAngularVelocityBody();
        Eigen::Vector3f gyro = _imuRocketCoordinateConverter.rotateRocketVectorToIMU(gyro_body);
        return hardware::IMU::Gyro{gyro.x(), gyro.y(), gyro.z()};
    };

private:
    IMURocketCoordinateConverter& _imuRocketCoordinateConverter;
    SimRocket &_simRocket;
};

} // namespace simHardware
