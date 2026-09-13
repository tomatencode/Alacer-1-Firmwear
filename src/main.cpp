#include <Arduino.h>

#include <etl/vector.h>

#include "./hardwear/radio/HC12.hpp"
#include "./hardwear/led/BlinkLed.hpp"
#include "./hardwear/buzzer/Buzzer.hpp"

Buzzer buzzer(PA8);
BlinkLed statusLed(PB14, 35, 50);
HC12 radioHC12(PB15, PA9, PA10);

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

    etl::vector<uint8_t, 32> receivedBytes;

    while (radioHC12.available() && receivedBytes.size() < receivedBytes.capacity()) {
        if (auto byte = radioHC12.read()) {
            receivedBytes.push_back(*byte);
        }
        statusLed.flash();
    }

    if (receivedBytes.size() == 1 && receivedBytes[0] == 0x41) {
        buzzer.beep(1000, 200);
    }

    if (!receivedBytes.empty()) {
        delay(10);
        radioHC12.send(
            std::span<const uint8_t>(receivedBytes.data(), receivedBytes.size())
        );
        receivedBytes.clear();
    }
}