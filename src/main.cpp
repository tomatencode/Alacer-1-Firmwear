#include <Arduino.h>

#include <vector>

#include "./hardwear/radio/HC12.hpp"
#include "./hardwear/led/BlinkLed.hpp"
#include "./hardwear/buzzer/Buzzer.hpp"

Buzzer buzzer(PA8);
BlinkLed statusLed(PB14, 255);
HC12 radioHC12(PB15, PA9, PA10, 9600, []() {statusLed.flash();});

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

    radioHC12.sendATCommand("AT+DEFAULT");
}

void loop() {
    buzzer.update();
    statusLed.update();

    std::vector<uint8_t> receivedBytes;
    while (radioHC12.available()) {
        if (auto byte = radioHC12.read()) {
            receivedBytes.push_back(*byte);
        }
        delay(5);
    }
    if (!receivedBytes.empty()) {
        delay(10);
        radioHC12.send(std::span(receivedBytes.data(), receivedBytes.size()));
        receivedBytes.clear();
    }
}