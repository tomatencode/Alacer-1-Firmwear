#include "LogManager.hpp"


LogManager::LogManager(StorageManager &storageManager)
    : _storageManager(storageManager) {
}

bool LogManager::startLog(const LogProtocol::LogMetadata &metadata, StorageManager::Filename filename) {
    bool fileCreated = _storageManager.startFile(filename);
    if (!fileCreated) {
        return false;
    }

    std::array<uint8_t, LogProtocol::headerSize> buffer;
    size_t encodedSize = LogProtocol::encodeHeader(metadata, buffer);
    StorageManager::WriteResult writeSuccess = _storageManager.write({buffer.data(), encodedSize});
    
    if (writeSuccess != StorageManager::WriteResult::Ok) {
        _storageManager.deleteFile(filename);
        return false;
    }

    _droppedEvents = 0;
    _lastTimestamp_us = micros();
    _startTime_ms = millis();
    _lastTimeSync_ms = _startTime_ms;
    

    return true;
}

bool LogManager::finishLog() {
    return _storageManager.finishFile();
}

void LogManager::addTimesyncIfNeeded() {
    if ((millis() - _lastTimeSync_ms) >= TIME_SYNC_INTERVAL_MS) {
        std::array<uint8_t, LogProtocol::maxEventSize> buffer;
        size_t encodedSize = LogProtocol::encodeTimeSync(millis() - _startTime_ms, buffer);
        StorageManager::WriteResult writeSuccess = _storageManager.write({buffer.data(), encodedSize});
        if (writeSuccess == StorageManager::WriteResult::Ok) {
            _lastTimeSync_ms = millis();
        }
    }
}

uint16_t LogManager::getCurrentTimestampDelta_us() {
    uint32_t currentTimestamp_us = micros();
    uint16_t delta = static_cast<uint16_t>(currentTimestamp_us - _lastTimestamp_us);
    _lastTimestamp_us = currentTimestamp_us;
    return delta;
}

void LogManager::appendDroppedEventsIfNeeded() {
    if (_droppedEvents > 0) {
        std::array<uint8_t, LogProtocol::maxEventSize> buffer;
        size_t encodedSize = LogProtocol::encodeDroppedEvents(_droppedEvents, millis() - _startTime_ms, buffer);
        StorageManager::WriteResult writeSuccess = _storageManager.write({buffer.data(), encodedSize});
        if (writeSuccess == StorageManager::WriteResult::Ok) {
            _droppedEvents = 0;
            _lastTimeSync_ms = millis();
        }
    }
}