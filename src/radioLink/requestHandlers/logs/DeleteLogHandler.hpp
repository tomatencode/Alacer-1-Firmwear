#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../logManagement/StorageManager.hpp"
#include "../../../helpers/codec/StringCodec.hpp"

namespace radioLink {

// DELETE_LOG request: length-prefixed filename, no trailing bytes.
// Empty SUCCESS response on deletion; empty FAILURE for malformed requests,
// unknown/open files, unmounted storage or flash errors. Does not reclaim space.
class DeleteLogHandler {
public:
    explicit DeleteLogHandler(logging::StorageManager& storageManager) : _storageManager(storageManager) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t>) {
        logging::StorageManager::Filename filename;
        const auto consumed = stringCodec::decode(payload, 0, filename);
        if (!consumed || filename.empty() || *consumed != payload.size() ||
            !_storageManager.isMounted()) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }
        const auto current = _storageManager.getCurrentFile();
        if (current && *current == filename) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }
        const bool deleted = _storageManager.deleteFile(filename);
        return {deleted ? MessageScheduler::HandlerResultStatus::SUCCESS
                        : MessageScheduler::HandlerResultStatus::FAILURE, 0};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<DeleteLogHandler, &DeleteLogHandler::handle>(*this);
    }

private:
    logging::StorageManager& _storageManager;
};

} // namespace radioLink
