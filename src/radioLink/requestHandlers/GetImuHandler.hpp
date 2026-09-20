#pragma once

#include "../MessageScheduler.hpp"
#include "../../hardware/imu/IMU.hpp"
#include "./helpers/FixedPointCodec.hpp"

class GetImuHandler : public MessageScheduler::RequestHandler {
public:
    explicit GetImuHandler(hardware::IMU& imu) : _imu(imu) {}

    void handleRequest(std::span<const uint8_t>, MessageScheduler::HandlerResult resultCallback) override {
        hardware::IMU::Accel accel = _imu.getAccel();
        hardware::IMU::Gyro gyro = _imu.getGyro();

        uint8_t payload[12];
        std::span<uint8_t> buffer(payload);
        constexpr float kScale = 1000.0f;
        fixedPoint::encode16(accel.x_m_s2, buffer, 0, kScale);
        fixedPoint::encode16(accel.y_m_s2, buffer, 2, kScale);
        fixedPoint::encode16(accel.z_m_s2, buffer, 4, kScale);
        fixedPoint::encode16(gyro.x_rad_s, buffer, 6, kScale);
        fixedPoint::encode16(gyro.y_rad_s, buffer, 8, kScale);
        fixedPoint::encode16(gyro.z_rad_s, buffer, 10, kScale);

        resultCallback(MessageScheduler::HandlerResultStatus::SUCCESS, payload);
    }

private:
    hardware::IMU& _imu;
};