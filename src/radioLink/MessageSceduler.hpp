#pragma once

#include <array>
#include <span>
#include "etl/delegate.h"
#include "etl/vector.h"
#include "etl/map.h"

#include "Protocol.hpp"
#include "../hardwear/radio/Radio.hpp"


constexpr uint32_t SEND_TIMEOUT_MS = 50;

class MessageScheduler {
public:
    using JobResult = etl::delegate<void(
        Protocol::MessageType responseType,
        std::span<const uint8_t> responsePayload)>;
    using Job = etl::delegate<void(
        std::span<const uint8_t> params,
        JobResult resultCallback)>;

    MessageScheduler(Protocol::Parser& parser, Radio& radio);

    void update();
    
    void registerForJob(Protocol::MessageType jobType, Job job);
    
    uint32_t getDroppedMessages() const { // for diagnostics
        return _dropedMessages;
    }
    
private:
    struct ResponseContext {
        MessageScheduler* scheduler;
        uint8_t sequenceId;
        bool inUse;

        void respond(
            Protocol::MessageType responseType,
            std::span<const uint8_t> responsePayload);
    };

    Protocol::Parser& _parser;
    Radio& _radio;

    void handleIncomingMessage(const Protocol::Message& message);
    void scheduleMessage(const Protocol::Message& message);

    uint32_t _lastReceived_ms;
    bool _didRespond;

    uint32_t _dropedMessages;

    etl::map<Protocol::MessageType, Job, 16> _jobs;
    etl::vector<Protocol::Message, 16> _scheduledMessages;
    std::array<ResponseContext, Protocol::MAX_MESSAGES_PER_FRAME> _responseContexts;
};