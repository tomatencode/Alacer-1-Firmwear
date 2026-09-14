#include <Arduino.h>
#include <array>
#include <span>

#include <etl/vector.h>

#include "./hardwear/radio/HC12.hpp"
#include "./hardwear/led/BlinkLed.hpp"
#include "./hardwear/buzzer/Buzzer.hpp"

#include "./radioLink/Protocol.hpp"
#include "./radioLink/MessageSceduler.hpp"


Buzzer buzzer(PA8);
BlinkLed statusLed(PB14, 35, 50);
HC12 radioHC12(PB15, PA9, PA10);

Protocol::Parser radiolinkParser;
MessageScheduler messageScheduler(radiolinkParser, radioHC12);

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

    messageScheduler.registerForJob(
        Protocol::MessageType::DO_BEEP,
        [](std::span<const uint8_t>, MessageScheduler::JobResult resultCallback) {
            buzzer.beep(1000, 200);
            resultCallback(MessageScheduler::JobResultStatus::SUCCESS, {});
        });

}

void loop() {
    buzzer.update();
    statusLed.update();
    radioHC12.update();
    messageScheduler.update();
}