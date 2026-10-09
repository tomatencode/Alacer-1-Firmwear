#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../logManagement/LogManager.hpp"

// IS_LOGGING: empty request; SUCCESS with one byte (1 = logging, 0 = idle).
class IsLoggingHandler {
public:
    explicit IsLoggingHandler(LogManager& logManager) : _logManager(logManager) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t> response) {
        if (!payload.empty() || response.empty()) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }
        response[0] = _logManager.isLogging() ? 1 : 0;
        return {MessageScheduler::HandlerResultStatus::SUCCESS, 1};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<IsLoggingHandler, &IsLoggingHandler::handle>(*this);
    }

private:
    LogManager& _logManager;
};