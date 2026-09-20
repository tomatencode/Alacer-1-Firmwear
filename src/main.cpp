#include <Arduino.h>
#include <array>
#include <span>

#include <etl/vector.h>

#include "./config/hardwearConstants.hpp"

#include "./hardware/led/BlinkLed.hpp"
#include "./hardware/buzzer/Buzzer.hpp"
#include "./hardware/radio/HC12.hpp"
#include "./hardware/imu/ICM45686.hpp"
#include "./hardware/barometer/MS5611.hpp"
#include "./hardware/Servo/Servo.hpp"
#include "./hardware/gimbal/Gimbal.hpp"

#include "./radioLink/Protocol.hpp"
#include "./radioLink/MessageScheduler.hpp"
#include "./radioLink/RequestHandlers.hpp"

#include "./locationEstimation/IMURocketCoordinateConverter.hpp"
#include "./locationEstimation/RotationAccumulator.hpp"

hardware::BlinkLed statusLed(PB14, 35, 50);
hardware::Buzzer buzzer(PA8);

hardware::Servo pitchServo(PA1, 2, 0.0f, 180.0f, 90.0f, 544, 2400);
hardware::Servo yawServo(PA2, 3, 0.0f, 180.0f, 90.0f, 544, 2400);

hardware::Gimbal gimbal(
    pitchServo, yawServo,
    hardware::Gimbal::GimbalPos{0.0f, 0.0f},
    -10.0f, 10.0f,
    -10.0f, 10.0f,
    (10.0f/40.0f), (15.0f/80.0f),
    115.0f, 75.0f
);

hardware::HC12 radioHC12(PB15, PA9, PA10);

Protocol::Parser radiolinkParser;
MessageScheduler messageScheduler(radiolinkParser, radioHC12);

SPIClass sensorSPI(PA7, PA6, PA5);
hardware::ICM45686 imu(PB10, sensorSPI);
hardware::MS5611 barometer(PB3, sensorSPI);

const Eigen::Quaternionf imuToRocketRotation =
    Eigen::AngleAxisf(IMU_ROLL_RAD, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(IMU_PITCH_RAD, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(IMU_YAW_RAD, Eigen::Vector3f::UnitZ());

IMURocketCoordinateConverter imuRocketConverter(
    Eigen::Vector3f{IMU_TO_ROCKET_X, IMU_TO_ROCKET_Y, IMU_TO_ROCKET_Z},
    imuToRocketRotation);

RotationAccumulator rotationAccumulator(imu, imuRocketConverter, 10000);

const hardware::Buzzer::Melody startupMelody = {
    {262, 200},
    {294, 200},
    {330, 200}
};

void setup() {
    statusLed.begin();
    buzzer.begin();

    radioHC12.begin();

    imu.begin();
    barometer.begin();

    pitchServo.begin();
    yawServo.begin();
    gimbal.begin();

    rotationAccumulator.setRotationQuaternion(Eigen::Quaternionf::Identity());

    requestHandlers::attachDoBeep(messageScheduler, &buzzer);
    requestHandlers::attachGetImu(messageScheduler, &imu);
    requestHandlers::attachGetBarometer(messageScheduler, &barometer);
    requestHandlers::attachGetGimbal(messageScheduler, &gimbal);
    requestHandlers::attachSetGimbal(messageScheduler, &gimbal);
    requestHandlers::attachGetRotation(messageScheduler, &rotationAccumulator);
    requestHandlers::attachSetRotation(messageScheduler, &rotationAccumulator);

    buzzer.playMelody(startupMelody);
}

void loop() {
    statusLed.update();
    buzzer.update();

    imu.update();
    barometer.update();

    rotationAccumulator.update();

    radioHC12.update();
    messageScheduler.update();
}