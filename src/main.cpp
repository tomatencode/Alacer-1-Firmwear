#include <Arduino.h>
#include <array>
#include <span>

#include <etl/vector.h>

#include "./hardwear/radio/HC12.hpp"
#include "./hardwear/led/BlinkLed.hpp"
#include "./hardwear/buzzer/Buzzer.hpp"

#include "./radioLink/Protocol.hpp"

Buzzer buzzer(PA8);
BlinkLed statusLed(PB14, 35, 50);
HC12 radioHC12(PB15, PA9, PA10);

Protocol::Parser radiolinkParser;

const Buzzer::Melody startupMelody = {
    {262, 200},
    {294, 200},
    {330, 200}
};

void setup() {
    statusLed.begin();
    radioHC12.begin();
    buzzer.begin();

    buzzer.playMelody(startupMelody);
}

void loop() {
    buzzer.update();
    statusLed.update();
    radioHC12.update();

    while (radioHC12.available()) {
        if (auto byte = radioHC12.read()) {
            radiolinkParser.feed(*byte);
        }
        statusLed.flash();
    }

    if (radiolinkParser.hasFrame()) {
        auto frame = radiolinkParser.takeFrame();
        if (frame.has_value()) {
            if (!frame->messages.empty()) {
                // Process the first message in the frame as an example
                auto& message = frame->messages[0];
                if (message.type == Protocol::MessageType::DO_BEEP) {
                    buzzer.beep(1000, 200);

                    Protocol::Frame responseFrame;
                    responseFrame.numMessages = 1;
                    Protocol::Message responseMessage;
                    responseMessage.type = Protocol::MessageType::DO_BEEP_ACK;
                    responseMessage.seqId = message.seqId;
                    responseMessage.messageLen = 0;
                    responseFrame.messages.push_back(responseMessage);

                    static std::array<uint8_t, Protocol::MAX_FRAME_SIZE> txBuffer;
                    if (auto encodedSize = Protocol::encode(responseFrame, txBuffer)) {
                        radioHC12.send(std::span<const uint8_t>(txBuffer.data(), *encodedSize));
                    }
                }
            }
        }
    }
}