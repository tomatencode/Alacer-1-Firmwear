#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../logManagement/LogManager.hpp"

// FINISH_LOG: empty request; SUCCESS/FAILURE with an empty response.
class FinishLogHandler {
public:
    explicit FinishLogHandler(LogManager& logManager) : _logManager(logManager) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t>) {
        if (!payload.empty()) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }
        const bool finished = _logManager.finishLog();
        return {finished ? MessageScheduler::HandlerResultStatus::SUCCESS
                         : MessageScheduler::HandlerResultStatus::FAILURE, 0};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<FinishLogHandler, &FinishLogHandler::handle>(*this);
    }

private:
    LogManager& _logManager;
};