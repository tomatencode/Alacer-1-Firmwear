#pragma once
#include "timing/Clock.hpp"

#include <cstdint>

namespace logging {
class LogManager;
}
class BarometricHeightCalculator;
class VerticalMovementTracker;
class HorizontalMovementTracker;
class RotationAccumulator;
namespace hardware {
class Battery;
class IMU;
class Barometer;
class IGimbal;
}

namespace logging {

// Samples raw and derived telemetry after sensors, estimators and control update.
// LogManager/LastEventsTracker suppress unchanged samples at on-disk precision.
// Discrete actions and driver health reporting remain at their source.
class TelemetryRecorder {
public:
    TelemetryRecorder(const Clock& clock, LogManager& logManager, hardware::Battery& battery,
                      hardware::IMU& imu, hardware::Barometer& barometer,
                      BarometricHeightCalculator& heightCalculator,
                      VerticalMovementTracker& verticalTracker,
                      HorizontalMovementTracker& horizontalTracker,
                      hardware::IGimbal& gimbal, RotationAccumulator& rotationAccumulator);

    void update();

private:
    const Clock& _clock;

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
    hardware::IGimbal& _gimbal;
    RotationAccumulator& _rotationAccumulator;
};

} // namespace logging
