#pragma once

#include <array>
#include <span>
#include "etl/delegate.h"
#include "etl/vector.h"
#include "etl/map.h"

#include "Protocol.hpp"
#include "../hardware/radio/Radio.hpp"


constexpr uint32_t SEND_TIMEOUT_MS = 50;

class MessageScheduler {
public:
    enum class HandlerResultStatus : uint8_t {
        SUCCESS,
        FAILURE,
    };

    using HandlerResult = etl::delegate<void(
        HandlerResultStatus status,
        std::span<const uint8_t> responsePayload)>;

    class RequestHandler {
    public:
        virtual ~RequestHandler() = default;
        virtual void handleRequest(std::span<const uint8_t> payload, MessageScheduler::HandlerResult resultCallback) = 0;
    };

    MessageScheduler(Protocol::Parser& parser, hardware::Radio& radio);

    void update();
    
    void registerRequestHandler(Protocol::MessageType requestType, RequestHandler& handler);
    
    uint32_t getDroppedMessages() const { // for diagnostics
        return _dropedMessages;
    }
    
private:
    struct ResponseContext {
        MessageScheduler* scheduler;
        Protocol::MessageType messageType;
        uint8_t sequenceId;
        bool inUse;

        void respond(
            HandlerResultStatus status,
            std::span<const uint8_t> responsePayload);
    };

    Protocol::Parser& _parser;
    hardware::Radio& _radio;

    void handleIncomingMessage(const Protocol::Message& message);
    void scheduleMessage(const Protocol::Message& message);

    uint32_t _lastReceived_ms;
    bool _didRespond;

    uint32_t _dropedMessages;

    etl::map<Protocol::MessageType, RequestHandler*, 32> _handlers;
    etl::vector<Protocol::Message, 32> _scheduledMessages;
    std::array<ResponseContext, 32> _responseContexts;
};