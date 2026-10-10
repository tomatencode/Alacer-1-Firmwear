#include "TelemetryRecorder.hpp"

#include <Arduino.h>
#include <cmath>

#include "LogManager.hpp"
#include "../hardwareIO/battery/Battery.hpp"
#include "../hardwareIO/imu/IMU.hpp"
#include "../hardwareIO/barometer/Barometer.hpp"
#include "../hardwareComponents/gimbal/Gimbal.hpp"
#include "../barometricHeightCalculation/BarometricHeightCalculator.hpp"
#include "../ascentTracking/VerticalMovementTracker.hpp"
#include "../ascentTracking/HorizontalMovementTracker.hpp"

TelemetryRecorder::TelemetryRecorder(LogManager& logManager, hardware::Battery& battery,
                                     hardware::IMU& imu, hardware::Barometer& barometer,
                                     BarometricHeightCalculator& heightCalculator,
                                     VerticalMovementTracker& verticalTracker,
                                     HorizontalMovementTracker& horizontalTracker,
                                     hardware::Gimbal& gimbal, RotationAccumulator& rotationAccumulator)
    : _logManager(logManager), _battery(battery), _imu(imu), _barometer(barometer),
      _heightCalculator(heightCalculator),
      _verticalTracker(verticalTracker), _horizontalTracker(horizontalTracker),
      _gimbal(gimbal), _rotationAccumulator(rotationAccumulator) {}

void TelemetryRecorder::update() {
    if (!_logManager.isLogging()) {
        _hasSample = false;
        return;
    }

    const uint32_t now_ms = millis();
    if (_hasSample && now_ms - _lastSample_ms < SAMPLE_INTERVAL_ms) return;
    _lastSample_ms = now_ms;
    _hasSample = true;

    appendIMUEvent();
    appendBarometerEvent();
    appendBatteryEvent();
    appendHorizontalMovementEvent();
    appendVerticalMovementEvent();
    appendGimbalEvent();
    appendRotationEvent();
}

void TelemetryRecorder::appendIMUEvent() {
    const auto accel = _imu.getAccel();
    const auto gyro = _imu.getGyro();
    _logManager.appendEvent(LogProtocol::IMUEvent{
        accel.x_m_s2, accel.y_m_s2, accel.z_m_s2,
        gyro.x_rad_s, gyro.y_rad_s, gyro.z_rad_s});
}

void TelemetryRecorder::appendBarometerEvent() {
    float pressure = _barometer.getPressure_Pa();
    float temperature = _barometer.getTemperature_C();
    _logManager.appendEvent(LogProtocol::BarometerEvent{
        pressure, temperature});
}

void TelemetryRecorder::appendBatteryEvent() {
    float voltage = _battery.getVoltage_v();
    _logManager.appendEvent(LogProtocol::BatteryEvent{
        voltage});
}

void TelemetryRecorder::appendGimbalEvent() {
    const auto gimbalPos = _gimbal.getTarget();
    _logManager.appendEvent(LogProtocol::GimbalEvent{
        gimbalPos.pitch_deg, gimbalPos.yaw_deg});
}

void TelemetryRecorder::appendRotationEvent() {
    const auto rotation = _rotationAccumulator.getRotationQuaternion();
    _logManager.appendEvent(LogProtocol::RotationEvent{rotation});
}

void TelemetryRecorder::appendHorizontalMovementEvent() {
    const auto horizontalMovement = _horizontalTracker.getTotalMovement_m();
    const auto horizontalVelocity = _horizontalTracker.getVelocity_m_s();
    const auto ascendStage = _horizontalTracker.getAscendStage();
    _logManager.appendEvent(LogProtocol::HorizontalMovementEvent{
        horizontalMovement[0], horizontalMovement[1],
        horizontalVelocity[0], horizontalVelocity[1],
        static_cast<uint8_t>(ascendStage)});
}

void TelemetryRecorder::appendVerticalMovementEvent() {
    float height = _verticalTracker.getHeight_m();
    float verticalVelocity = _verticalTracker.getVelocity_m_s();
    _logManager.appendEvent(LogProtocol::VerticalMovementEvent{
        height, verticalVelocity});
}