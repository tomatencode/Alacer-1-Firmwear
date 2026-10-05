#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <array>

#include <ArduinoEigen.h>
#include "etl/vector.h"

#include "StorageManager.hpp"

struct LogMetadata {
    uint32_t timestamp_unix;
    std::string log_name;

    Eigen::Quaternionf initialRotation;
    Eigen::Quaternionf targetAngle;

    float pidKp;
    float pidKi;
    float pidKd;

    float initialHeight_m;
};

enum class EventType : uint8_t {
    IMU_DATA,
    GYRO_DATA
};

struct LogEvent {
    EventType type;
    std::span<const uint8_t> data;
};

class LogManager {
public:
    LogManager(StorageManager &storageManager);

    bool startLog(const LogMetadata &metadata);
    std::optional<uint16_t> finishLog(); // returns the ID of the finished log's file if successful, std::nullopt otherwise

    void appendEvent(LogEvent event);

    void update();
private:

    static constexpr size_t EVENT_BUFFER_SIZE = 1024;
    static constexpr size_t ENCODED_EVENT_MAX_SIZE = 1024;

    etl::vector<uint8_t, ENCODED_EVENT_MAX_SIZE> encodeEvent(LogEvent event);

    etl::vector<LogEvent, EVENT_BUFFER_SIZE> _eventBuffer;

    StorageManager &_storageManager;
};