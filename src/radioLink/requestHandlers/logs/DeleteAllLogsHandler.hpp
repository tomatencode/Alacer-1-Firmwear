#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../logManagement/StorageManager.hpp"

// DELETE_ALL_LOGS: empty request and response. SUCCESS also for an empty store.
// FAILURE for nonempty payloads, unmounted storage, active logging or flash errors.
// Logically removes all files and reclaims space; this is not a secure flash wipe.
class DeleteAllLogsHandler {
public:
    explicit DeleteAllLogsHandler(StorageManager& storageManager) : _storageManager(storageManager) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t>) {
        if (!payload.empty()) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }
        const bool deleted = _storageManager.deleteAllFiles();
        return {deleted ? MessageScheduler::HandlerResultStatus::SUCCESS
                        : MessageScheduler::HandlerResultStatus::FAILURE, 0};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<DeleteAllLogsHandler, &DeleteAllLogsHandler::handle>(*this);
    }

private:
    StorageManager& _storageManager;
};