#include <Arduino.h>
#include <array>
#include <numbers>
#include <span>

#include <etl/vector.h>

#include "./config/hardwearConstants.hpp"

#include "./hardwareIO/led/BlinkLed.hpp"
#include "./hardwareIO/buzzer/Buzzer.hpp"
#include "./hardwareIO/radio/HC12.hpp"
#include "./hardwareIO/imu/ICM45686.hpp"
#include "./hardwareIO/barometer/MS5611.hpp"
#include "./hardwareIO/Servo/Servo.hpp"
#include "./hardwareIO/pyro/PyroManager.hpp"
#include "./hardwareIO/pyro/PyroChannel.hpp"

#include "./hardwareComponents/gimbal/Gimbal.hpp"
#include "./hardwareComponents/motor/MotorIgniter.hpp"
#include "./hardwareComponents/parashoot/Parashoot.hpp"

#include "./radioLink/Protocol.hpp"
#include "./radioLink/MessageScheduler.hpp"
#include "./radioLink/requestHandlers/DoBeepHandler.hpp"
#include "./radioLink/requestHandlers/GetImuHandler.hpp"
#include "./radioLink/requestHandlers/GetBarometerHandler.hpp"
#include "./radioLink/requestHandlers/GetGimbalHandler.hpp"
#include "./radioLink/requestHandlers/SetGimbalHandler.hpp"
#include "./radioLink/requestHandlers/GetRotationHandler.hpp"
#include "./radioLink/requestHandlers/SetRotationHandler.hpp"

#include "./rotationEstimation/IMURocketCoordinateConverter.hpp"
#include "./rotationEstimation/RotationAccumulator.hpp"

#include "./acentTracking/VerticalMovementTracker.hpp"

#include "./stateManagement/FlightStateManager.hpp"

hardware::BlinkLed statusLed(PB14, 35, 50);
hardware::Buzzer buzzer(PA8);

hardware::Servo pitchServo(PA1, 2, 0.0f, 180.0f, 90.0f, 544, 2400);
hardware::Servo yawServo(PA2, 3, 0.0f, 180.0f, 90.0f, 544, 2400);

PyroManager pyroManager(PC13);
PyroChannel pyroChanel1(PB5, PB4, pyroManager);
PyroChannel pyroChanel2(PB6, PB7, pyroManager);
PyroChannel pyroChanel3(PB8, PB9, pyroManager);

hardware::HC12 radioHC12(PB15, PA9, PA10);

SPIClass sensorSPI(PA7, PA6, PA5);

hardware::ICM45686 imu(PB10, sensorSPI);

hardware::MS5611 barometer(PB3, sensorSPI);

MotorIgniter motorIgniter;
Parachute parashoot;

hardware::Gimbal gimbal(
    pitchServo, yawServo,
    hardware::Gimbal::GimbalPos{0.0f, 0.0f},
    -10.0f, 10.0f,
    -10.0f, 10.0f,
    (10.0f/40.0f), (15.0f/80.0f),
    115.0f, 75.0f
);

Protocol::Parser radiolinkParser;
MessageScheduler messageScheduler(radiolinkParser, radioHC12);

const Eigen::Quaternionf imuToRocketRotation =
    Eigen::AngleAxisf(IMU_ROLL_DEG * std::numbers::pi_v<float> / 180.0f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(IMU_PITCH_DEG * std::numbers::pi_v<float> / 180.0f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(IMU_YAW_DEG * std::numbers::pi_v<float> / 180.0f, Eigen::Vector3f::UnitZ());

IMURocketCoordinateConverter imuRocketConverter(
    Eigen::Vector3f{IMU_TO_ROCKET_X, IMU_TO_ROCKET_Y, IMU_TO_ROCKET_Z},
    imuToRocketRotation);

RotationAccumulator rotationAccumulator(imu, imuRocketConverter, 10000);

BarometricHeightCalculator barometricHeightCalculator(barometer);

VerticalMovementTracker verticalMovementTracker(barometricHeightCalculator);

FlightStateManager flightStateManager(rotationAccumulator, verticalMovementTracker, motorIgniter, parashoot);

// Radio Request handlers
DoBeepHandler beepHandler(buzzer);
GetImuHandler imuHandler(imu);
GetBarometerHandler barometerHandler(barometer);
GetGimbalHandler getGimbalHandler(gimbal);
SetGimbalHandler setGimbalHandler(gimbal);
GetRotationHandler getRotationHandler(rotationAccumulator);
SetRotationHandler setRotationHandler(rotationAccumulator);

const hardware::Buzzer::Melody startupMelody = {
    {262, 200},
    {294, 200},
    {330, 200}
};

void setup() {
    statusLed.begin();
    buzzer.begin();

    radioHC12.begin();

    pyroManager.begin();
    pyroChanel1.begin();
    pyroChanel2.begin();
    pyroChanel3.begin();

    imu.begin();
    barometer.begin();

    pitchServo.begin();
    yawServo.begin();
    gimbal.begin();

    parashoot.setPyroChannel(pyroChanel1);
    motorIgniter.setPyroChannel(pyroChanel2);

    rotationAccumulator.setRotationQuaternion(Eigen::Quaternionf::Identity());

    messageScheduler.registerRequestHandler(Protocol::MessageType::DO_BEEP, beepHandler);
    messageScheduler.registerRequestHandler(Protocol::MessageType::GET_IMU, imuHandler);
    messageScheduler.registerRequestHandler(Protocol::MessageType::GET_BAROMETER, barometerHandler);
    messageScheduler.registerRequestHandler(Protocol::MessageType::GET_GIMBAL, getGimbalHandler);
    messageScheduler.registerRequestHandler(Protocol::MessageType::SET_GIMBAL, setGimbalHandler);
    messageScheduler.registerRequestHandler(Protocol::MessageType::GET_ROTATION, getRotationHandler);
    messageScheduler.registerRequestHandler(Protocol::MessageType::SET_ROTATION, setRotationHandler);

    buzzer.playMelody(startupMelody);
}

void loop() {
    statusLed.update();
    buzzer.update();

    imu.update();
    barometer.update();

    pyroChanel1.update();
    pyroChanel2.update();
    pyroChanel3.update();

    rotationAccumulator.update();
    verticalMovementTracker.update();

    flightStateManager.update();

    radioHC12.update();
    messageScheduler.update();
}