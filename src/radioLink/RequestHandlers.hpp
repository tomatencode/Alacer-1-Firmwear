#pragma once

#include "./Protocol.hpp"
#include "./MessageScheduler.hpp"
#include "./hardware/buzzer/Buzzer.hpp"
#include "./hardware/imu/ICM45686.hpp"
#include "./hardware/barometer/Barometer.hpp"
#include "./hardware/gimbal/Gimbal.hpp"
#include "./locationEstimation/RotationAccumulator.hpp"

namespace requestHandlers {

void attachDoBeep(MessageScheduler& messageScheduler, hardware::Buzzer* buzzer);
void attachGetImu(MessageScheduler& messageScheduler, hardware::ICM45686* imu);
void attachGetBarometer(MessageScheduler& messageScheduler, hardware::Barometer* barometer);
void attachGetGimbal(MessageScheduler& messageScheduler, hardware::Gimbal* gimbal);
void attachSetGimbal(MessageScheduler& messageScheduler, hardware::Gimbal* gimbal);
void attachGetRotation(MessageScheduler& messageScheduler, RotationAccumulator* rotationAccumulator);
void attachSetRotation(MessageScheduler& messageScheduler, RotationAccumulator* rotationAccumulator);

} // namespace requestHandlers