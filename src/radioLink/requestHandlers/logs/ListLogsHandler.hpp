#pragma once

#include <algorithm>
#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../logManagement/StorageManager.hpp"
#include "../../../helpers/codec/StringCodec.hpp"

// LIST_LOGS: empty request starts at index 0; otherwise one uint8 start index.
// SUCCESS response: [total files, next index, page count], followed by
// length-prefixed filenames. next index == total files means end of list.
// Lists StorageManager::listFiles(): completed/recovered files, not the open log.
// Pages are capped at 255 bytes (the radio message length is uint8).
// Invalid index, unmounted storage or insufficient response space => FAILURE.
class ListLogsHandler {
public:
    explicit ListLogsHandler(StorageManager& storageManager) : _storageManager(storageManager) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t> response) {
        if (payload.size() > 1 || response.size() < 3 || !_storageManager.isMounted()) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }
        const auto& files = _storageManager.listFiles();
        const size_t start = payload.empty() ? 0 : payload[0];
        if (start > files.size()) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }
        response = response.first(std::min(response.size(), size_t{255}));
        size_t offset = 3;
        size_t next = start;
        while (next < files.size()) {
            const auto written = stringCodec::encode(files[next], response, offset);
            if (!written) {
                break;
            }
            offset += *written;
            ++next;
        }
        if (next == start && start < files.size()) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }
        response[0] = static_cast<uint8_t>(files.size());
        response[1] = static_cast<uint8_t>(next);
        response[2] = static_cast<uint8_t>(next - start);
        return {MessageScheduler::HandlerResultStatus::SUCCESS, offset};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<ListLogsHandler, &ListLogsHandler::handle>(*this);
    }

private:
    StorageManager& _storageManager;
};