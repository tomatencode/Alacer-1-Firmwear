#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../logManagement/LogManager.hpp"
#include "../../../helpers/codec/LittleEndianCodec.hpp"
#include "../../../helpers/codec/FixedPointCodec.hpp"
#include "../../../helpers/codec/QuaternionCodec.hpp"
#include "../../../helpers/codec/StringCodec.hpp"

namespace radioLink {

// START_LOG payload:
//   [0..] length-prefixed filename (1..32 bytes), no trailing bytes.
// The filename is also the log name; the configuration is logged as a FlightConfigurationEvent.
// Response: SUCCESS/FAILURE with an empty payload.
class StartLogHandler {
public:
    explicit StartLogHandler(logging::LogManager& logManager) : _logManager(logManager) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t>) {
        logging::StorageManager::Filename filename;
        const auto filenameSize = stringCodec::decode(payload, 0, filename);
        if (!filenameSize || filename.empty() || payload.size() != *filenameSize) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        const bool started = _logManager.startLog(filename);
        return {started ? MessageScheduler::HandlerResultStatus::SUCCESS
                        : MessageScheduler::HandlerResultStatus::FAILURE, 0};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<StartLogHandler, &StartLogHandler::handle>(*this);
    }

private:
    logging::LogManager& _logManager;
};

} // namespace radioLink
