#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../hardwareIO/imu/IMU.hpp"
#include "../FixedPointCodec.hpp"

class GetImuHandler {
public:
    explicit GetImuHandler(hardware::IMU& imu) : _imu(imu) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t>, std::span<uint8_t> response) {
        hardware::IMU::Accel accel = _imu.getAccel();
        hardware::IMU::Gyro gyro = _imu.getGyro();

        fixedPoint::encode16(accel.x_m_s2, response, 0);
        fixedPoint::encode16(accel.y_m_s2, response, 2);
        fixedPoint::encode16(accel.z_m_s2, response, 4);
        fixedPoint::encode16(gyro.x_rad_s, response, 6);
        fixedPoint::encode16(gyro.y_rad_s, response, 8);
        fixedPoint::encode16(gyro.z_rad_s, response, 10);

        return {MessageScheduler::HandlerResultStatus::SUCCESS, 12};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<GetImuHandler, &GetImuHandler::handle>(*this);
    }

private:
    hardware::IMU& _imu;
};
