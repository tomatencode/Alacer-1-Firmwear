#include <Arduino.h>

#include <vector>

#define STATUS_LED PB14

HardwareSerial HC12(PA12, PA11);

void setup() {
    pinMode(STATUS_LED, OUTPUT); // Set the built-in LED pin as an output
    HC12.begin(9600);

    delay(100);

    HC12.write("AT+DEFAULT\r\n");
}

void loop() {
    std::vector<uint8_t> receivedBytes;
    while (HC12.available()) {
        receivedBytes.push_back(HC12.read());
        analogWrite(STATUS_LED, 35);
        delay(5);
    }
    if (!receivedBytes.empty()) {
        delay(50);
        HC12.write(receivedBytes.data(), receivedBytes.size());
        receivedBytes.clear();
        analogWrite(STATUS_LED, 0);
    }
}