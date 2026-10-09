#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include "etl/delegate.h"
#include "etl/vector.h"
#include "etl/map.h"

#include "Protocol.hpp"
#include "../hardwareIO/radio/Radio.hpp"

class MessageScheduler {
public:
    enum class HandlerResultStatus : uint8_t {
        SUCCESS,
        FAILURE,
    };

    struct HandlerResult {
        HandlerResultStatus status;
        size_t responseLength = 0;
    };

    // Synchronous request handler:
    // - requestPayload: incoming request bytes (valid only during the call).
    // - responseBuffer: scratch buffer (256 bytes) to write the response into.
    // - return: status + number of valid bytes in responseBuffer.
    using RequestHandler = etl::delegate<HandlerResult(std::span<const uint8_t> requestPayload, std::span<uint8_t> responseBuffer)>;

    MessageScheduler(Protocol::Parser& parser, hardware::Radio& radio, etl::delegate<bool()> isMidFlightCb = {});

    void setMidFlightCallback(etl::delegate<bool()> isMidFlightCb) { _isMidFlightCb = isMidFlightCb; }

    void update();

    struct HandlerOptions {
        bool groundOnly = true; // fail-closed: must opt OUT with {.groundOnly = false}
    };

    void registerRequestHandler(Protocol::MessageType requestType, RequestHandler handler, HandlerOptions options);
    void registerRequestHandler(Protocol::MessageType requestType, RequestHandler handler); // groundOnly=true

    uint32_t getDroppedMessages() const { // for diagnostics
        return _droppedMessages;
    }

private:
    Protocol::Parser& _parser;
    hardware::Radio& _radio;

    void handleIncomingMessage(const Protocol::Message& message);
    void scheduleMessage(const Protocol::Message& message);

    bool _didRespond;

    uint32_t _droppedMessages;

    etl::map<Protocol::MessageType, RequestHandler, 48> _handlers;
    etl::map<Protocol::MessageType, bool, 48> _groundOnlyHandlers;
    etl::delegate<bool()> _isMidFlightCb;
    etl::vector<Protocol::Message, 32> _scheduledMessages;
};
