#pragma once

#include <cstdint>
#include <span>

#include <ArduinoEigen.h>

namespace LogProtocol {

constexpr uint16_t headerSize = 256;
constexpr uint16_t maxEventSize = 128;

// Header layout (256 bytes, zero-padded):
//   0..3   UNIX timestamp (uint32)
//   4..19  initial rotation quaternion (x, y, z, w)
//   20..35 target angle quaternion (x, y, z, w)
//   36..47 PID kp, ki, kd
//   48..51 initial height in metres
// Quaternion and float fields are int32 fixed-point values scaled by 100.
//
// Record layout:
//   byte 0    event type
//   bytes 1..2 timestamp delta in microseconds for sampled events
//   remaining event-specific payload, encoded little-endian
enum class EventType : uint8_t {
    TimeSync = 0x01,
    DroppedEvents = 0x02,
    UpdateCycleDone = 0x03,
    IMU = 0x04,
    Barometer = 0x05,
    Battery = 0x06,
    Gimbal = 0x07,
    Rotation = 0x08,
    HorizontalMovement = 0x09,
    VerticalMovement = 0x0A,
};

struct LogMetadata {
    uint32_t timestamp_unix;

    Eigen::Quaternionf initialRotation;
    Eigen::Quaternionf targetAngle;

    float pidKp;
    float pidKi;
    float pidKd;

    float initialHeight_m;
};

struct TimeSyncEvent {
    uint32_t absolute_timestamp_ms;
};

struct DroppedEvents {
    uint32_t count;
};

struct IMUEvent {
    float accel_x_m_s2;
    float accel_y_m_s2;
    float accel_z_m_s2;

    float gyro_x_rad_s;
    float gyro_y_rad_s;
    float gyro_z_rad_s;
};

struct BarometerEvent {
    float pressure_Pa;
    float temperature_C;
};

struct BatteryEvent {
    float voltage_v;
};

struct GimbalEvent {
    float pitch_deg;
    float yaw_deg;
};

struct RotationEvent {
    Eigen::Quaternionf rotation;
};

struct HorizontalMovementEvent {
    float x_m;
    float y_m;
    float velocity_x_m_s;
    float velocity_y_m_s;
    uint8_t ascentStage;
};

struct VerticalMovementEvent {
    float height_m;
    float velocity_m_s;
};

struct UpdateCycleDoneEvent {
};

size_t encodeHeader(const LogMetadata& metadata, std::span<uint8_t> buffer);

// TimeSync: event type followed by uint32 milliseconds since log start.
size_t encodeTimeSync(uint32_t time_since_start_ms, std::span<uint8_t> buffer);
// DroppedEvents: event type, uint32 milliseconds since log start, uint32 count.
size_t encodeDroppedEvents(uint32_t count, uint32_t time_since_start_ms, std::span<uint8_t> buffer);


// UpdateCycleDone: event type only. to track main loop frequency.
size_t encodeEvent(const UpdateCycleDoneEvent& event, uint16_t timestamp_d_us, std::span<uint8_t> buffer);
// IMU: event type, uint16 delta microseconds, then six int32 fixed-point values.
size_t encodeEvent(const IMUEvent& event, uint16_t timestamp_d_us, std::span<uint8_t> buffer);
// Barometer: event type, uint16 delta microseconds, then two int32 fixed-point values.
size_t encodeEvent(const BarometerEvent& event, uint16_t timestamp_d_us, std::span<uint8_t> buffer);
// Battery: event type, uint16 delta microseconds, then one int32 fixed-point voltage.
size_t encodeEvent(const BatteryEvent& event, uint16_t timestamp_d_us, std::span<uint8_t> buffer);
// Gimbal: event type, uint16 delta microseconds, then pitch and yaw as int32 fixed-point values.
size_t encodeEvent(const GimbalEvent& event, uint16_t timestamp_d_us, std::span<uint8_t> buffer);
// Rotation: event type, uint16 delta microseconds, then quaternion (x, y, z, w) as four int32 fixed-point values.
size_t encodeEvent(const RotationEvent& event, uint16_t timestamp_d_us, std::span<uint8_t> buffer);
// HorizontalMovement: event type, uint16 delta microseconds, four int32 fixed-point values
// (x, y, velocity x, velocity y), then a uint8 ascent stage.
size_t encodeEvent(const HorizontalMovementEvent& event, uint16_t timestamp_d_us, std::span<uint8_t> buffer);
// VerticalMovement: event type, uint16 delta microseconds, then height and velocity as int32 fixed-point values.
size_t encodeEvent(const VerticalMovementEvent& event, uint16_t timestamp_d_us, std::span<uint8_t> buffer);

} // namespace LogProtocol