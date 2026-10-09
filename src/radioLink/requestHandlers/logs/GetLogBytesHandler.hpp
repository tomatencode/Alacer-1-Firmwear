#pragma once

#include <algorithm>
#include <span>

#include "GetLogInfoHandler.hpp"

// GET_LOG_BYTES request: length-prefixed filename, uint32 LE payload offset,
// uint16 LE requested length (1..MAX_CHUNK_BYTES), no trailing bytes.
// SUCCESS response: uint32 LE offset followed by raw log bytes (no flash framing).
// Requests near EOF are shortened; offset == size returns only the offset.
// Invalid ranges/files, insufficient buffers and read/CRC errors => empty FAILURE.
// Stateless and retry-safe; use one outstanding request on the client initially.
class GetLogBytesHandler {
public:
    static constexpr size_t RESPONSE_HEADER_SIZE = 4;

    explicit GetLogBytesHandler(StorageManager& storageManager) : _storageManager(storageManager) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t> response) {
        StorageManager::Filename filename;
        const auto consumed = stringCodec::decode(payload, 0, filename);
        if (!consumed || filename.empty() || payload.size() != *consumed + 6) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }
        const uint32_t offset = littleEndian::decodeU32(payload, *consumed);
        const uint16_t length = littleEndian::decodeU16(payload, *consumed + 4);
        if (length == 0 || length > GetLogInfoHandler::MAX_CHUNK_BYTES) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }
        const auto size = _storageManager.fileSizePayloadChecked(filename);
        if (!size || offset > *size) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }
        const size_t expected = std::min(uint32_t{length}, *size - offset);
        if (response.size() < RESPONSE_HEADER_SIZE + expected) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }
        if (expected > 0 && _storageManager.readFile(filename, offset,
                response.subspan(RESPONSE_HEADER_SIZE, expected)) != expected) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }
        littleEndian::encodeU32(offset, response, 0);
        return {MessageScheduler::HandlerResultStatus::SUCCESS, RESPONSE_HEADER_SIZE + expected};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<GetLogBytesHandler, &GetLogBytesHandler::handle>(*this);
    }

private:
    StorageManager& _storageManager;
};