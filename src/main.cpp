#include <Arduino.h>
#include <array>
#include <numbers>
#include <span>

#include <etl/vector.h>

#include "./config/IMUPos.hpp"
#include "./config/GimbalGears.hpp"

#include "./hardwareIO/led/BlinkLed.hpp"
#include "./hardwareIO/buzzer/Buzzer.hpp"
#include "./hardwareIO/radio/HC12.hpp"
#include "./hardwareIO/imu/ICM45686.hpp"
#include "./hardwareIO/barometer/MS5611.hpp"
#include "./hardwareIO/servo/Servo.hpp"
#include "./hardwareIO/pyro/PyroManager.hpp"
#include "./hardwareIO/pyro/PyroChannel.hpp"

#include "./hardwareComponents/gimbal/Gimbal.hpp"
#include "./hardwareComponents/motor/MotorIgniter.hpp"
#include "./hardwareComponents/parachute/Parachute.hpp"

#include "./radioLink/Protocol.hpp"
#include "./radioLink/MessageScheduler.hpp"
#include "./radioLink/requestHandlers/buzzer/DoBeepHandler.hpp"
#include "./radioLink/requestHandlers/sensors/GetImuHandler.hpp"
#include "./radioLink/requestHandlers/sensors/GetBarometerHandler.hpp"
#include "./radioLink/requestHandlers/gimbal/GetGimbalHandler.hpp"
#include "./radioLink/requestHandlers/gimbal/SetGimbalHandler.hpp"
#include "./radioLink/requestHandlers/rotation/GetRotationHandler.hpp"
#include "./radioLink/requestHandlers/rotation/SetRotationHandler.hpp"
#include "./radioLink/requestHandlers/pyro/FirePyroHandler.hpp"
#include "./radioLink/requestHandlers/pyro/GetPyroContinuityHandler.hpp"
#include "./radioLink/requestHandlers/pyro/GetPyroSoftwareArmedHandler.hpp"
#include "./radioLink/requestHandlers/pyro/SetPyroSoftwareArmedHandler.hpp"
#include "./radioLink/requestHandlers/pyro/GetPyroHardwareArmedHandler.hpp"
#include "./radioLink/requestHandlers/sensors/GetBaroHeightHandler.hpp"
#include "./radioLink/requestHandlers/sensors/CalibrateBaroHeightHandler.hpp"
#include "./radioLink/requestHandlers/flight/AbortFlightHandler.hpp"
#include "./radioLink/requestHandlers/flight/EndFlightHandler.hpp"
#include "./radioLink/requestHandlers/flight/GetFlightStateHandler.hpp"
#include "./radioLink/requestHandlers/flight/GetCountdownTimeHandler.hpp"
#include "./radioLink/requestHandlers/flight/GetFlightLocationHandler.hpp"
#include "./radioLink/requestHandlers/led/FlashLedHandler.hpp"

#include "./rotationEstimation/IMURocketCoordinateConverter.hpp"
#include "./rotationEstimation/RotationAccumulator.hpp"

#include "./controlPID/ControlPID.hpp"

#include "./ascentTracking/VerticalMovementTracker.hpp"
#include "./ascentTracking/HorizontalMovementTracker.hpp"

#include "./stateManagement/FlightStateManager.hpp"

hardware::BlinkLed statusLed(PB14, 35, 50);
hardware::Buzzer buzzer(PA8);

hardware::Servo pitchServo(PA1, 2, 0.0f, 180.0f, 90.0f, 544, 2400, 180.0);
hardware::Servo yawServo(PA2, 3, 0.0f, 180.0f, 90.0f, 544, 2400, 180.0);

PyroChannel pyroChannel1(PB5, PB4);
PyroChannel pyroChannel2(PB6, PB7);
PyroChannel pyroChannel3(PB8, PB9);

std::array<PyroChannel*, 3> pyroChannels = {
    &pyroChannel1,
    &pyroChannel2,
    &pyroChannel3
};

PyroManager pyroManager(PC13, pyroChannels);

hardware::HC12 radioHC12(PB15, PA9, PA10);

SPIClass sensorSPI(PA7, PA6, PA5);

hardware::ICM45686 imu(PB10, sensorSPI);

hardware::MS5611 barometer(PB3, sensorSPI);

MotorIgniter motorIgniter;
Parachute parachute;

hardware::Gimbal gimbal(
    pitchServo, yawServo,
    hardware::Gimbal::GimbalPos{0.0f, 0.0f},
    -GIMBAL_LIMIT_deg, GIMBAL_LIMIT_deg,
    -GIMBAL_LIMIT_deg, GIMBAL_LIMIT_deg,
    GIMBAL_PITCH_GEAR_RATIO, GIMBAL_YAW_GEAR_RATIO,
    GIMBAL_PITCH_SERVO_OFFSET_deg, GIMBAL_YAW_SERVO_OFFSET_deg
);

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
HorizontalMovementTracker horizontalMovementTracker(verticalMovementTracker, rotationAccumulator);

ControlPID controlPID(rotationAccumulator, gimbal);

FlightStateManager flightStateManager(controlPID, rotationAccumulator, barometricHeightCalculator, verticalMovementTracker, horizontalMovementTracker, pyroManager, motorIgniter, parachute);

Protocol::Parser radioLinkParser;
MessageScheduler messageScheduler(radioLinkParser, radioHC12);

// Radio Request handlers
DoBeepHandler beepHandler(buzzer);
GetImuHandler imuHandler(imu);
GetBarometerHandler barometerHandler(barometer);
GetGimbalHandler getGimbalHandler(gimbal);
SetGimbalHandler setGimbalHandler(gimbal);
GetRotationHandler getRotationHandler(rotationAccumulator);
SetRotationHandler setRotationHandler(rotationAccumulator);
FirePyroHandler firePyroHandler(pyroManager.getPyroChannels());
GetPyroContinuityHandler getPyroContinuityHandler(pyroManager.getPyroChannels());
GetPyroSoftwareArmedHandler getPyroSoftwareArmedHandler(pyroManager);
SetPyroSoftwareArmedHandler setPyroSoftwareArmedHandler(pyroManager);
GetPyroHardwareArmedHandler getPyroHardwareArmedHandler(pyroManager);
AbortFlightHandler abortFlightHandler(flightStateManager);
EndFlightHandler endFlightHandler(flightStateManager);
GetBaroHeightHandler baroHeightHandler(barometricHeightCalculator);
CalibrateBaroHeightHandler calibrateBaroHeightHandler(barometricHeightCalculator);
GetFlightLocationHandler flightLocationHandler(verticalMovementTracker, horizontalMovementTracker);
GetFlightStateHandler flightStateHandler(flightStateManager);
GetCountdownTimeHandler countdownTimeHandler(flightStateManager);
FlashLedHandler flashLedHandler(statusLed);

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
    pyroChannel1.begin();
    pyroChannel2.begin();
    pyroChannel3.begin();

    imu.begin();
    barometer.begin();

    pitchServo.begin();
    yawServo.begin();
    gimbal.begin();

    controlPID.setPIDParameters(1.0f, 0.0f, 0.0f); // Example PID parameters
    controlPID.setTarget(Eigen::Quaternionf::Identity()); // Example target angle

    parachute.setPyroChannel(pyroChannel1);
    motorIgniter.setPyroChannel(pyroChannel2);

    rotationAccumulator.setRotationQuaternion(Eigen::Quaternionf::Identity());

    messageScheduler.setMidFlightCallback(
        etl::delegate<bool()>::create<FlightStateManager, &FlightStateManager::isMidFlight>(flightStateManager));

    messageScheduler.registerRequestHandler(Protocol::MessageType::DO_BEEP, beepHandler.callback());
    messageScheduler.registerRequestHandler(Protocol::MessageType::GET_IMU, imuHandler.callback(), {.groundOnly = false});
    messageScheduler.registerRequestHandler(Protocol::MessageType::GET_BAROMETER, barometerHandler.callback(), {.groundOnly = false});
    messageScheduler.registerRequestHandler(Protocol::MessageType::GET_GIMBAL, getGimbalHandler.callback(), {.groundOnly = false});
    messageScheduler.registerRequestHandler(Protocol::MessageType::SET_GIMBAL, setGimbalHandler.callback());
    messageScheduler.registerRequestHandler(Protocol::MessageType::GET_ROTATION, getRotationHandler.callback(), {.groundOnly = false});
    messageScheduler.registerRequestHandler(Protocol::MessageType::SET_ROTATION, setRotationHandler.callback());
    messageScheduler.registerRequestHandler(Protocol::MessageType::FIRE_PYRO, firePyroHandler.callback());
    messageScheduler.registerRequestHandler(Protocol::MessageType::GET_PYRO_CONTINUITY, getPyroContinuityHandler.callback(), {.groundOnly = false});
    messageScheduler.registerRequestHandler(Protocol::MessageType::GET_PYRO_SOFTWARE_ARMED, getPyroSoftwareArmedHandler.callback(), {.groundOnly = false});
    messageScheduler.registerRequestHandler(Protocol::MessageType::SET_PYRO_SOFTWARE_ARMED, setPyroSoftwareArmedHandler.callback());
    messageScheduler.registerRequestHandler(Protocol::MessageType::GET_PYRO_HARDWARE_ARMED, getPyroHardwareArmedHandler.callback(), {.groundOnly = false});
    messageScheduler.registerRequestHandler(Protocol::MessageType::ABORT_FLIGHT, abortFlightHandler.callback(), {.groundOnly = false});
    messageScheduler.registerRequestHandler(Protocol::MessageType::END_FLIGHT, endFlightHandler.callback(), {.groundOnly = false});
    messageScheduler.registerRequestHandler(Protocol::MessageType::GET_BARO_HEIGHT, baroHeightHandler.callback(), {.groundOnly = false});
    messageScheduler.registerRequestHandler(Protocol::MessageType::CALIBRATE_BARO_HEIGHT, calibrateBaroHeightHandler.callback());
    messageScheduler.registerRequestHandler(Protocol::MessageType::GET_FLIGHT_LOCATION, flightLocationHandler.callback(), {.groundOnly = false});
    messageScheduler.registerRequestHandler(Protocol::MessageType::GET_FLIGHT_STATE, flightStateHandler.callback(), {.groundOnly = false});
    messageScheduler.registerRequestHandler(Protocol::MessageType::GET_COUNTDOWN_TIME, countdownTimeHandler.callback(), {.groundOnly = false});
    messageScheduler.registerRequestHandler(Protocol::MessageType::FLASH_LED, flashLedHandler.callback(), {.groundOnly = false});

    buzzer.playMelody(startupMelody);
}

void loop() {
    statusLed.update();
    buzzer.update();

    imu.update();
    barometer.update();

    pyroChannel1.update();
    pyroChannel2.update();
    pyroChannel3.update();

    rotationAccumulator.update();
    verticalMovementTracker.update();
    horizontalMovementTracker.update();

    flightStateManager.update();

    radioHC12.update();
    messageScheduler.update();
    controlPID.update();
}