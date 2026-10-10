#pragma once

#include <span>

#include "../../MessageScheduler.hpp"
#include "../../../logManagement/LogManager.hpp"
#include "../../../helpers/codec/LittleEndianCodec.hpp"
#include "../../../helpers/codec/FixedPointCodec.hpp"
#include "../../../helpers/codec/QuaternionCodec.hpp"
#include "../../../helpers/codec/StringCodec.hpp"

// START_LOG payload:
//   [0..3] UNIX timestamp, little-endian uint32
//   [4..19] initial rotation, [20..35] target angle (x,y,z,w)
//   [36..47] PID kp, ki, kd; [48..51] initial height in metres
//   Quaternion/float fields use int32 fixed-point scaled by 100.
//   [52..] length-prefixed filename (1..32 bytes), no trailing bytes.
// The filename is also the log name; the configuration is logged as a FlightConfigurationEvent.
// Response: SUCCESS/FAILURE with an empty payload.
class StartLogHandler {
public:
    static constexpr size_t METADATA_SIZE = 52;

    explicit StartLogHandler(LogManager& logManager) : _logManager(logManager) {}

    MessageScheduler::HandlerResult handle(std::span<const uint8_t> payload, std::span<uint8_t>) {
        StorageManager::Filename filename;
        const auto filenameSize = stringCodec::decode(payload, METADATA_SIZE, filename);
        if (!filenameSize || filename.empty() || payload.size() != METADATA_SIZE + *filenameSize) {
            return {MessageScheduler::HandlerResultStatus::FAILURE, 0};
        }

        LogProtocol::LogMetadata metadata;
        metadata.timestamp_unix = littleEndian::decodeU32(payload, 0);
        metadata.logName = filename;

        LogProtocol::FlightConfigurationEvent configuration;
        configuration.initialRotation = quaternionCodec::decode(payload, 4);
        configuration.targetAngle = quaternionCodec::decode(payload, 20);
        configuration.pidKp = fixedPoint::decode32(payload, 36);
        configuration.pidKi = fixedPoint::decode32(payload, 40);
        configuration.pidKd = fixedPoint::decode32(payload, 44);
        configuration.initialHeight_m = fixedPoint::decode32(payload, 48);

        const bool started = _logManager.startLog(metadata, configuration, filename);
        return {started ? MessageScheduler::HandlerResultStatus::SUCCESS
                        : MessageScheduler::HandlerResultStatus::FAILURE, 0};
    }

    MessageScheduler::RequestHandler callback() {
        return MessageScheduler::RequestHandler::create<StartLogHandler, &StartLogHandler::handle>(*this);
    }

private:
    LogManager& _logManager;
};