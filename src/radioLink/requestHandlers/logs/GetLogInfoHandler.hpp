#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../logManagement/StorageManager.hpp"
#include "../../../helpers/codec/StringCodec.hpp"
#include "../../../helpers/codec/LittleEndianCodec.hpp"

// GET_LOG_INFO request: length-prefixed filename, no trailing bytes.
// SUCCESS response: uint32 LE verified payload size, uint16 LE maximum chunk size.
// Only closed/recovered files are downloadable. Invalid requests/files or storage
// read/CRC errors return FAILURE with an empty payload.
class GetLogInfoHandler {
public:
    static constexpr uint16_t MAX_CHUNK_BYTES = 240;
    static constexpr size_t RESPONSE_SIZE = 6;

    explicit GetLogInfoHandler(StorageManager& storageManager) : _storageManager(storageManager) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t> response) {
        StorageManager::Filename filename;
        const auto consumed = stringCodec::decode(payload, 0, filename);
        if (!consumed || filename.empty() || *consumed != payload.size() || response.size() < RESPONSE_SIZE) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }
        const auto size = _storageManager.fileSizePayloadChecked(filename);
        if (!size) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }
        littleEndian::encodeU32(*size, response, 0);
        littleEndian::encodeU16(MAX_CHUNK_BYTES, response, 4);
        return {MessageScheduler::HandlerResultStatus::SUCCESS, RESPONSE_SIZE};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<GetLogInfoHandler, &GetLogInfoHandler::handle>(*this);
    }

private:
    StorageManager& _storageManager;
};