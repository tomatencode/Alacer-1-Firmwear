#pragma once

#include <cstdint>

class LogManager;
class BarometricHeightCalculator;
class VerticalMovementTracker;
class HorizontalMovementTracker;
class RotationAccumulator;
namespace hardware {
class Battery;
class IMU;
class Barometer;
class Gimbal;
}

// Samples raw and derived telemetry after sensors, estimators and control update.
// LogManager/LastEventsTracker suppress unchanged samples at on-disk precision.
// Discrete actions and driver health reporting remain at their source.
class TelemetryRecorder {
public:
    TelemetryRecorder(LogManager& logManager, hardware::Battery& battery,
                      hardware::IMU& imu, hardware::Barometer& barometer,
                      BarometricHeightCalculator& heightCalculator,
                      VerticalMovementTracker& verticalTracker,
                      HorizontalMovementTracker& horizontalTracker,
                      hardware::Gimbal& gimbal, RotationAccumulator& rotationAccumulator);

    void update();

private:

    void appendIMUEvent();
    void appendBarometerEvent();
    void appendBatteryEvent();
    void appendHorizontalMovementEvent();
    void appendVerticalMovementEvent();
    void appendGimbalEvent();
    void appendRotationEvent();

    static constexpr uint32_t SAMPLE_INTERVAL_ms = 20; // 50 Hz maximum

    uint32_t _lastSample_ms = 0;
    uint32_t _logSession = 0;
    bool _hasSample = false;

    LogManager& _logManager;
    hardware::Battery& _battery;
    hardware::IMU& _imu;
    hardware::Barometer& _barometer;
    BarometricHeightCalculator& _heightCalculator;
    VerticalMovementTracker& _verticalTracker;
    HorizontalMovementTracker& _horizontalTracker;
    hardware::Gimbal& _gimbal;
    RotationAccumulator& _rotationAccumulator;
};