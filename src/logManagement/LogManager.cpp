#include "LogManager.hpp"


LogManager::LogManager(StorageManager &storageManager)
    : _storageManager(storageManager) {
}

bool LogManager::startLog(const LogProtocol::LogMetadata &metadata, const LogProtocol::FlightConfigurationEvent &configuration,
                          StorageManager::Filename filename) {
    if (_storageManager.isFileOpen()) {
        return false; // finish the previous log first; don't orphan it
    }
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
    // First appendEvent() anchors both clocks with dt=0, so the idle gap
    // between startLog() and the first sample cannot wrap the 16-bit delta.
    _hasTimestamp = false;
    _lastTimestampUs = 0;
    _startTime_ms = millis();
    _lastTimeSync_ms = _startTime_ms;
    _lastEventsTracker.clearLasts();

    if (!appendEvent(configuration)) {
        _storageManager.finishFile();
        _storageManager.deleteFile(filename);
        return false;
    }

    return true;
}

bool LogManager::finishLog() {
    if (!_storageManager.isFileOpen()) {
        return false;
    }
    const uint32_t nowMs = millis();
    appendDroppedEventsIfNeeded(nowMs);
    addTimesyncIfNeeded(nowMs, true); // final anchor even if interval not due
    return _storageManager.finishFile();
}

bool LogManager::writeTimeSync(uint32_t nowMs) {
    size_t encodedSize = LogProtocol::encodeTimeSync(nowMs - _startTime_ms, _scratch);
    StorageManager::WriteResult writeSuccess =
        _storageManager.write({_scratch.data(), encodedSize});
    if (writeSuccess == StorageManager::WriteResult::Ok) {
        _lastTimeSync_ms = nowMs;
        return true;
    }
    return false;
}

void LogManager::addTimesyncIfNeeded(uint32_t nowMs, bool force) {
    if (!force && (nowMs - _lastTimeSync_ms) < TIME_SYNC_INTERVAL_MS) {
        return;
    }
    writeTimeSync(nowMs);
}

void LogManager::appendDroppedEventsIfNeeded(uint32_t nowMs) {
    if (_droppedEvents == 0) {
        return;
    }
    const uint32_t pending = _droppedEvents;
    size_t encodedSize = LogProtocol::encodeDroppedEvents(pending, nowMs - _startTime_ms, _scratch);
    StorageManager::WriteResult writeSuccess =
        _storageManager.write({_scratch.data(), encodedSize});
    if (writeSuccess == StorageManager::WriteResult::Ok) {
        _droppedEvents -= pending; // only clear what we reported
        _lastTimeSync_ms = nowMs; // marker carries an absolute time reference
    }
}