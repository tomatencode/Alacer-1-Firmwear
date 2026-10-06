#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <array>

#include <ArduinoEigen.h>
#include "etl/vector.h"

#include "StorageManager.hpp"
#include "LogProtocol.hpp"


class LogManager {
public:
    LogManager(StorageManager &storageManager);

    bool startLog(const LogProtocol::LogMetadata &metadata, StorageManager::Filename filename);
    bool finishLog();

    template<typename T>
    void appendEvent(const T& event) {
        if (!_storageManager.isFileOpen())
            return;

        appendDroppedEventsIfNeeded();
        addTimesyncIfNeeded();

        uint16_t dt = getCurrentTimestampDelta_us();

        static std::array<uint8_t, LogProtocol::maxEventSize> buffer;

        size_t encodedSize = LogProtocol::encodeEvent(event, dt, buffer);

        auto res = _storageManager.write(
            std::span<const uint8_t>(buffer.data(), encodedSize));
        if (res != StorageManager::WriteResult::Ok)
            _droppedEvents++;
    }

private:

    static constexpr uint32_t TIME_SYNC_INTERVAL_MS = 50; // before 16 bit us delta timestamp overflows

    uint32_t _droppedEvents = 0;

    uint16_t _lastTimestamp_us = 0;
    uint32_t _lastTimeSync_ms = 0;
    uint32_t _startTime_ms = 0;

    void appendDroppedEventsIfNeeded();
    void addTimesyncIfNeeded();
    uint16_t getCurrentTimestampDelta_us();

    StorageManager &_storageManager;
};