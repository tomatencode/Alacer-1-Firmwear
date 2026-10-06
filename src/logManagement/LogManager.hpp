#pragma once

#include <array>
#include <cstdint>
#include <span>

#include <Arduino.h>

#include "StorageManager.hpp"
#include "LogProtocol.hpp"


class LogManager {
public:
    LogManager(StorageManager &storageManager);

    bool startLog(const LogProtocol::LogMetadata &metadata, StorageManager::Filename filename);
    bool finishLog();

    bool isLogging() const { return _storageManager.isFileOpen(); }

    template<typename T>
    bool appendEvent(const T& event) {
        if (!_storageManager.isFileOpen())
            return false;

        const uint32_t nowUs = micros();
        const uint32_t nowMs = millis();

        uint16_t dt;
        if (!_hasTimestamp) {
            _lastTimestampUs = nowUs;
            _hasTimestamp = true;
            dt = 0;
        } else {
            const uint32_t gap = nowUs - _lastTimestampUs;
            if (gap > kMaxDeltaUs) {
                writeTimeSync(nowMs);
                _lastTimestampUs = nowUs;
                dt = 0;
            } else {
                _lastTimestampUs = nowUs;
                dt = static_cast<uint16_t>(gap);
            }
        }

        appendDroppedEventsIfNeeded(nowMs);
        addTimesyncIfNeeded(nowMs, false);

        const size_t encodedSize = LogProtocol::encodeEvent(event, dt, _scratch);

        const auto res = _storageManager.write(
            std::span<const uint8_t>(_scratch.data(), encodedSize));
        if (res != StorageManager::WriteResult::Ok) {
            ++_droppedEvents; // single count: only the main event counts
            return false;
        }
        return true;
    }

private:

    static constexpr uint32_t TIME_SYNC_INTERVAL_MS = 50; // before 16 bit us delta timestamp overflows
    static constexpr uint32_t kMaxDeltaUs = 60000; // margin below 65535 wrap

    uint32_t _droppedEvents = 0;

    uint32_t _lastTimestampUs = 0;
    bool _hasTimestamp = false;
    uint32_t _lastTimeSync_ms = 0;
    uint32_t _startTime_ms = 0;

    std::array<uint8_t, LogProtocol::maxEventSize> _scratch{};

    void appendDroppedEventsIfNeeded(uint32_t nowMs);
    void addTimesyncIfNeeded(uint32_t nowMs, bool force);
    bool writeTimeSync(uint32_t nowMs);

    StorageManager &_storageManager;
};