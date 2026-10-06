#pragma once

#include <cstdint>
#include <span>

#include <ArduinoEigen.h>

namespace LogProtocol {

constexpr uint16_t headerSize = 256;
constexpr uint16_t maxEventSize = 128;

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
    float x_m_s2;
    float y_m_s2;
    float z_m_s2;

    float roll_rad;
    float pitch_rad;
    float yaw_rad;
};

struct BarometerEvent {
    float pressure;
    float temperature;
};

size_t encodeHeader(const LogMetadata& metadata, std::span<uint8_t> buffer);

size_t encodeTimeSync(uint32_t time_since_start_ms, std::span<uint8_t> buffer);
size_t encodeDroppedEvents(uint32_t count, uint32_t time_since_start_ms, std::span<uint8_t> buffer);

size_t encodeEvent(const IMUEvent& event, uint16_t timestamp_d_us, std::span<uint8_t> buffer);
size_t encodeEvent(const BarometerEvent& event, uint16_t timestamp_d_us, std::span<uint8_t> buffer);


} // namespace LogProtocol