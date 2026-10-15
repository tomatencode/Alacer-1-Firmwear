#pragma once
#include <cstdint>
#include <cstddef>

#include "simMode/SimRocket.hpp"
#include "hardwareIO/imu/IMU.hpp"

namespace simHardware {

class SimIMU : public hardware::IMU {
public:
    explicit SimIMU(SimRocket &simRocket);

    void begin() override;
    void update() override;

    hardware::IMU::Accel getAccel() const override;
    hardware::IMU::Gyro getGyro() const override;

    bool isConnected() const;

private:
    SimRocket &_simRocket;
};

} // namespace simHardware
