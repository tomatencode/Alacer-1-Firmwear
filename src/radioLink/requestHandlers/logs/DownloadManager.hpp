#pragma once

#include <Arduino.h>
#include <algorithm>
#include <array>
#include <optional>
#include <span>
#include <utility>

// One ground-only download session; no allocation or speculative prefetch.
// All integers are little-endian; malformed requests return empty FAILURE.
// START_LOG_DOWNLOAD: filename (StringCodec), u32 client token.
//   SUCCESS: u32 session, u32 payload size, u16 chunk bytes, u32 chunk count.
//   Repeating the same filename/token while active returns the same session.
//   A different start fails until stop/timeout; token should be unique per start.
// GET_LOG_CHUNK: u32 session, u32 zero-based chunk index.
//   SUCCESS: u32 session, u32 index, raw bytes (last chunk may be shorter).
//   Forward requests advance the reader; backward requests use RAM or readFile.
// STOP_LOG_DOWNLOAD: u32 session. SUCCESS: empty, including repeat of last stop.
// Sessions remain active after EOF so the final response can be retried.
class DownloadManager {
public:
    static constexpr size_t START_RESPONSE_SIZE = 14;
    static constexpr size_t CHUNK_HEADER_SIZE = 8;
    static constexpr uint16_t CHUNK_BYTES = 240;
    static constexpr uint32_t TIMEOUT_MS = 60000;
    static_assert(CHUNK_HEADER_SIZE + CHUNK_BYTES <= 255);

    explicit DownloadManager(StorageManager& storage) : _storage(storage) {}

    void update() {
        const uint32_t now = millis();
        if (_reader && (!_reader->valid() || uint32_t{now - _lastActivity} >= TIMEOUT_MS)) {
            clear();
        }
    }

    MessageScheduler::HandlerResult start(std::span<const uint8_t> payload,
                                          std::span<uint8_t> response) {
        update();
        StorageManager::Filename filename;
        const auto consumed = stringCodec::decode(payload, 0, filename);
        if (!consumed || filename.empty() || payload.size() != *consumed + 4 ||
            response.size() < START_RESPONSE_SIZE) {
            return failure();
        }
        const uint32_t token = littleEndian::decodeU32(payload, *consumed);
        if (_reader) {
            if (filename != _filename || token != _clientToken) {
                return failure();
            }
        } else {
            auto reader = _storage.openSequentialReader(filename);
            if (!reader) {
                return failure();
            }
            _reader = std::move(reader);
            _filename = filename;
            _clientToken = token;
            // Never recycle a session ID during this manager's lifetime.
            if (_nextSession == 0) {
                clear();
                return failure();
            }
            _session = _nextSession++;
            _chunkCount = _reader->size() / CHUNK_BYTES +
                          (_reader->size() % CHUNK_BYTES != 0);
            _cachedIndex.reset();
        }
        _lastActivity = millis(); // validation scan can take time
        littleEndian::encodeU32(_session, response, 0);
        littleEndian::encodeU32(_reader->size(), response, 4);
        littleEndian::encodeU16(CHUNK_BYTES, response, 8);
        littleEndian::encodeU32(_chunkCount, response, 10);
        return {MessageScheduler::HandlerResultStatus::SUCCESS, START_RESPONSE_SIZE};
    }

    MessageScheduler::HandlerResult getChunk(std::span<const uint8_t> payload,
                                             std::span<uint8_t> response) {
        update();
        if (payload.size() != 8 || !_reader ||
            littleEndian::decodeU32(payload, 0) != _session) {
            return failure();
        }
        const uint32_t index = littleEndian::decodeU32(payload, 4);
        if (index >= _chunkCount) {
            return failure();
        }
        const uint32_t offset = index * uint32_t{CHUNK_BYTES};
        const size_t count = std::min(uint32_t{CHUNK_BYTES}, _reader->size() - offset);
        if (response.size() < CHUNK_HEADER_SIZE + count) {
            return failure();
        }
        if (!_cachedIndex || *_cachedIndex != index) {
            _cachedIndex.reset(); // partially failed reads must never become retries
            auto output = std::span(_chunk).first(count);
            bool ok;
            if (offset < _reader->position()) {
                ok = _reader->copyCached(offset, output) ||
                     _storage.readFile(_filename, offset, output) == count;
            } else {
                ok = _reader->advanceTo(offset) && _reader->read(output) == count;
            }
            if (!ok) {
                clear(); // cursor may have advanced partially; require restart
                return failure();
            }
            _cachedIndex = index;
        }
        std::copy_n(_chunk.begin(), count, response.begin() + CHUNK_HEADER_SIZE);
        littleEndian::encodeU32(_session, response, 0);
        littleEndian::encodeU32(index, response, 4);
        _lastActivity = millis();
        return {MessageScheduler::HandlerResultStatus::SUCCESS, CHUNK_HEADER_SIZE + count};
    }

    MessageScheduler::HandlerResult stop(std::span<const uint8_t> payload,
                                         std::span<uint8_t>) {
        update();
        if (payload.size() != 4) {
            return failure();
        }
        const uint32_t session = littleEndian::decodeU32(payload, 0);
        if (_reader) {
            if (session != _session) {
                return failure();
            }
            _lastStopped = session;
            clear();
        } else if (session == 0 || session != _lastStopped) {
            return failure();
        }
        return {MessageScheduler::HandlerResultStatus::SUCCESS, 0};
    }

    MessageScheduler::RequestHandler startCallback() {
        return MessageScheduler::RequestHandler::create<DownloadManager, &DownloadManager::start>(*this);
    }
    MessageScheduler::RequestHandler chunkCallback() {
        return MessageScheduler::RequestHandler::create<DownloadManager, &DownloadManager::getChunk>(*this);
    }
    MessageScheduler::RequestHandler stopCallback() {
        return MessageScheduler::RequestHandler::create<DownloadManager, &DownloadManager::stop>(*this);
    }

private:
    static MessageScheduler::HandlerResult failure() {
        return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
    }
    void clear() {
        _reader.reset();
        _cachedIndex.reset();
    }

    StorageManager& _storage;
    std::optional<StorageManager::SequentialFileReader> _reader;
    StorageManager::Filename _filename;
    uint32_t _clientToken = 0;
    uint32_t _session = 0;
    uint32_t _nextSession = 1;
    uint32_t _lastStopped = 0;
    uint32_t _chunkCount = 0;
    uint32_t _lastActivity = 0;
    std::optional<uint32_t> _cachedIndex;
    std::array<uint8_t, CHUNK_BYTES> _chunk{};
};