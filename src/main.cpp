#include <Arduino.h>

#define STATUS_LED PB14

void setup() {
    pinMode(STATUS_LED, OUTPUT); // Set the built-in LED pin as an output
}

void loop() {
    analogWrite(STATUS_LED, 35); // Turn the LED on
    delay(1000); // Wait for 1 second
    analogWrite(STATUS_LED, 0); // Turn the LED off
    delay(1000); // Wait for 1 second
}