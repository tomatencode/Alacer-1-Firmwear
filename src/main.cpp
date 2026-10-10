#include <Arduino.h>
#include <array>
#include <span>

#include <etl/vector.h>

#include "./config/IMUPos.hpp"
#include "./config/GimbalGears.hpp"

#include "./hardwareIO/led/BlinkLed.hpp"
#include "./hardwareIO/buzzer/Buzzer.hpp"
#include "./hardwareIO/battery/Battery.hpp"
#include "./hardwareIO/radio/HC12.hpp"
#include "./hardwareIO/imu/ICM45686.hpp"
#include "./hardwareIO/barometer/MS5611.hpp"
#include "./hardwareIO/servo/Servo.hpp"
#include "./hardwareIO/pyro/PyroArmManager.hpp"
#include "./hardwareIO/pyro/PyroChannel.hpp"
#include "./hardwareIO/flash/W25Q32JV.hpp"

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
#include "./radioLink/requestHandlers/rotation/SetAccumulatingRotationHandler.hpp"
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
#include "./radioLink/requestHandlers/flight/StartCountdownHandler.hpp"
#include "./radioLink/requestHandlers/flight/RetryDeployParachuteHandler.hpp"
#include "./radioLink/requestHandlers/controlPID/SetPIDParametersHandler.hpp"
#include "./radioLink/requestHandlers/controlPID/GetPIDParametersHandler.hpp"
#include "./radioLink/requestHandlers/controlPID/SetControllingHandler.hpp"
#include "./radioLink/requestHandlers/controlPID/GetControllingHandler.hpp"
#include "./radioLink/requestHandlers/controlPID/SetPIDTargetHandler.hpp"
#include "./radioLink/requestHandlers/controlPID/GetPIDTargetHandler.hpp"
#include "./radioLink/requestHandlers/led/FlashLedHandler.hpp"
#include "./radioLink/requestHandlers/battery/GetBatteryVoltageHandler.hpp"
#include "./radioLink/requestHandlers/logs/StartLogHandler.hpp"
#include "./radioLink/requestHandlers/logs/FinishLogHandler.hpp"
#include "./radioLink/requestHandlers/logs/IsLoggingHandler.hpp"
#include "./radioLink/requestHandlers/logs/ListLogsHandler.hpp"
#include "./radioLink/requestHandlers/logs/GetLogSizeHandler.hpp"
#include "./radioLink/requestHandlers/logs/DeleteLogHandler.hpp"
#include "./radioLink/requestHandlers/logs/DeleteAllLogsHandler.hpp"
#include "./radioLink/requestHandlers/logs/DownloadManager.hpp"

#include "./logManagement/StorageManager.hpp"
#include "./logManagement/LogManager.hpp"
#include "./logManagement/TelemetryRecorder.hpp"

#include "./helpers/coordinates/IMURocketCoordinateConverter.hpp"
#include "./rotationEstimation/RotationAccumulator.hpp"

#include "./controlPID/ControlPID.hpp"

#include "./ascentTracking/VerticalMovementTracker.hpp"
#include "./ascentTracking/HorizontalMovementTracker.hpp"

#include "./flightStateManagement/FlightStateManager.hpp"

hardware::BlinkLed statusLed(PB14, 35, 50);

hardware::Buzzer buzzer(PA8);

hardware::Battery battery(PB1);

hardware::HC12 radioHC12(PB15, PA9, PA10);

SPIClass spiBus(PA7, PA6, PA5);

hardware::W25Q32JV flashMemory(PB12, spiBus);

hardware::ICM45686 imu(PB10, spiBus);

hardware::MS5611 barometer(PB3, spiBus);

hardware::Servo pitchServo(PA1, 2, 0.0f, 180.0f, 90.0f, 544, 2400, 180.0);
hardware::Servo yawServo(PA2, 3, 0.0f, 180.0f, 90.0f, 544, 2400, 180.0);

PyroArmManager pyroArmManager(PC13);

PyroChannel pyroChannel1(PB5, PB4, pyroArmManager);
PyroChannel pyroChannel2(PB6, PB7, pyroArmManager);
PyroChannel pyroChannel3(PB8, PB9, pyroArmManager);

std::array<PyroChannel*, 3> pyroChannels = {
    &pyroChannel1,
    &pyroChannel2,
    &pyroChannel3
};

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

StorageManager storageManager(flashMemory);
LogManager logManager(storageManager);

const Eigen::Matrix3f imuToRocketRotationMatrix = (Eigen::Matrix3f() <<
    IMU_TO_ROCKET_ROTATION_MATRIX[0][0], IMU_TO_ROCKET_ROTATION_MATRIX[0][1], IMU_TO_ROCKET_ROTATION_MATRIX[0][2],
    IMU_TO_ROCKET_ROTATION_MATRIX[1][0], IMU_TO_ROCKET_ROTATION_MATRIX[1][1], IMU_TO_ROCKET_ROTATION_MATRIX[1][2],
    IMU_TO_ROCKET_ROTATION_MATRIX[2][0], IMU_TO_ROCKET_ROTATION_MATRIX[2][1], IMU_TO_ROCKET_ROTATION_MATRIX[2][2]).finished();

const Eigen::Quaternionf imuToRocketRotation(imuToRocketRotationMatrix);

IMURocketCoordinateConverter imuRocketConverter(
    Eigen::Vector3f{IMU_TO_ROCKET_X, IMU_TO_ROCKET_Y, IMU_TO_ROCKET_Z},
    imuToRocketRotation);

RotationAccumulator rotationAccumulator(imu, imuRocketConverter, 10000);

BarometricHeightCalculator barometricHeightCalculator(barometer);

VerticalMovementTracker verticalMovementTracker(barometricHeightCalculator);
HorizontalMovementTracker horizontalMovementTracker(verticalMovementTracker, rotationAccumulator);

ControlPID controlPID(rotationAccumulator, gimbal);

FlightStateManager flightStateManager(
    logManager,
    controlPID,
    rotationAccumulator,
    barometricHeightCalculator,
    verticalMovementTracker,
    horizontalMovementTracker,
    motorIgniter,
    parachute
);

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
SetAccumulatingRotationHandler setAccumulatingRotationHandler(rotationAccumulator);
FirePyroHandler firePyroHandler(pyroChannels);
GetPyroContinuityHandler getPyroContinuityHandler(pyroChannels);
GetPyroSoftwareArmedHandler getPyroSoftwareArmedHandler(pyroArmManager);
SetPyroSoftwareArmedHandler setPyroSoftwareArmedHandler(pyroArmManager);
GetPyroHardwareArmedHandler getPyroHardwareArmedHandler(pyroArmManager);
AbortFlightHandler abortFlightHandler(flightStateManager);
EndFlightHandler endFlightHandler(flightStateManager);
GetBaroHeightHandler baroHeightHandler(barometricHeightCalculator);
CalibrateBaroHeightHandler calibrateBaroHeightHandler(barometricHeightCalculator);
GetFlightLocationHandler flightLocationHandler(verticalMovementTracker, horizontalMovementTracker);
GetFlightStateHandler flightStateHandler(flightStateManager);
GetCountdownTimeHandler countdownTimeHandler(flightStateManager);
StartCountdownHandler startCountdownHandler(flightStateManager, pyroChannels);
RetryDeployParachuteHandler retryDeployParachuteHandler(flightStateManager);
SetPIDParametersHandler setPIDParametersHandler(controlPID);
GetPIDParametersHandler getPIDParametersHandler(controlPID);
SetControllingHandler setControllingHandler(controlPID);
GetControllingHandler getControllingHandler(controlPID);
SetPIDTargetHandler setPIDTargetHandler(controlPID);
GetPIDTargetHandler getPIDTargetHandler(controlPID);
FlashLedHandler flashLedHandler(statusLed);
GetBatteryVoltageHandler getBatteryVoltageHandler(battery);
StartLogHandler startLogHandler(logManager);
FinishLogHandler finishLogHandler(logManager);
IsLoggingHandler isLoggingHandler(logManager);
ListLogsHandler listLogsHandler(storageManager);
GetLogSizeHandler getLogSizeHandler(storageManager);
DeleteLogHandler deleteLogHandler(storageManager);
DeleteAllLogsHandler deleteAllLogsHandler(storageManager);
DownloadManager downloadManager(storageManager);

TelemetryRecorder telemetryRecorder(
    logManager, battery,
    imu, barometer,
    barometricHeightCalculator,
    verticalMovementTracker,
    horizontalMovementTracker,
    gimbal, rotationAccumulator
);

const hardware::Buzzer::Melody startupMelody = {
    {262, 200},
    {294, 200},
    {330, 200}
};

void setup() {
    statusLed.begin();
    buzzer.begin();

    battery.begin();

    radioHC12.begin();

    pyroArmManager.begin();
    pyroChannel1.begin();
    pyroChannel2.begin();
    pyroChannel3.begin();

    imu.begin();
    barometer.begin();
    flashMemory.begin();

    pitchServo.begin();
    yawServo.begin();
    gimbal.begin();

    storageManager.begin();

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
    messageScheduler.registerRequestHandler(Protocol::MessageType::GET_BATTERY_VOLTAGE, getBatteryVoltageHandler.callback());
    messageScheduler.registerRequestHandler(Protocol::MessageType::GET_ROTATION, getRotationHandler.callback(), {.groundOnly = false});
    messageScheduler.registerRequestHandler(Protocol::MessageType::SET_ROTATION, setRotationHandler.callback());
    messageScheduler.registerRequestHandler(Protocol::MessageType::SET_ACCUMULATING_ROTATION, setAccumulatingRotationHandler.callback());
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
    messageScheduler.registerRequestHandler(Protocol::MessageType::START_COUNTDOWN, startCountdownHandler.callback());
    messageScheduler.registerRequestHandler(Protocol::MessageType::RETRY_DEPLOY_PARACHUTE, retryDeployParachuteHandler.callback());
    messageScheduler.registerRequestHandler(Protocol::MessageType::SET_PID_PARAMETERS, setPIDParametersHandler.callback());
    messageScheduler.registerRequestHandler(Protocol::MessageType::GET_PID_PARAMETERS, getPIDParametersHandler.callback(), {.groundOnly = false});
    messageScheduler.registerRequestHandler(Protocol::MessageType::SET_CONTROLLING, setControllingHandler.callback());
    messageScheduler.registerRequestHandler(Protocol::MessageType::GET_CONTROLLING, getControllingHandler.callback(), {.groundOnly = false});
    messageScheduler.registerRequestHandler(Protocol::MessageType::SET_PID_TARGET, setPIDTargetHandler.callback());
    messageScheduler.registerRequestHandler(Protocol::MessageType::GET_PID_TARGET, getPIDTargetHandler.callback(), {.groundOnly = false});
    messageScheduler.registerRequestHandler(Protocol::MessageType::FLASH_LED, flashLedHandler.callback(), {.groundOnly = false});
    messageScheduler.registerRequestHandler(Protocol::MessageType::START_LOG, startLogHandler.callback());
    messageScheduler.registerRequestHandler(Protocol::MessageType::FINISH_LOG, finishLogHandler.callback());
    messageScheduler.registerRequestHandler(Protocol::MessageType::IS_LOGGING, isLoggingHandler.callback(), {.groundOnly = false});
    messageScheduler.registerRequestHandler(Protocol::MessageType::LIST_LOGS, listLogsHandler.callback(), {.groundOnly = false});
    messageScheduler.registerRequestHandler(Protocol::MessageType::GET_LOG_SIZE, getLogSizeHandler.callback());
    messageScheduler.registerRequestHandler(Protocol::MessageType::DELETE_LOG, deleteLogHandler.callback());
    messageScheduler.registerRequestHandler(Protocol::MessageType::DELETE_ALL_LOGS, deleteAllLogsHandler.callback());
    messageScheduler.registerRequestHandler(Protocol::MessageType::START_LOG_DOWNLOAD, downloadManager.startCallback());
    messageScheduler.registerRequestHandler(Protocol::MessageType::GET_LOG_CHUNK, downloadManager.chunkCallback());
    messageScheduler.registerRequestHandler(Protocol::MessageType::STOP_LOG_DOWNLOAD, downloadManager.stopCallback());

    buzzer.playMelody(startupMelody);
}

void loop() {
    downloadManager.update();
    statusLed.update();
    buzzer.update();

    imu.update();
    barometer.update();

    pyroChannel1.update();
    pyroChannel2.update();
    pyroChannel3.update();

    storageManager.update();

    rotationAccumulator.update();
    verticalMovementTracker.update();
    horizontalMovementTracker.update();

    flightStateManager.update();

    radioHC12.update();
    messageScheduler.update();
    controlPID.update();

    logManager.appendEvent(LogProtocol::UpdateCycleDoneEvent{});
}