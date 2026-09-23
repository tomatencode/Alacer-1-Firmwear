#include "PyroChanel.hpp"

#include <Arduino.h>

PyroChanel::PyroChanel(int mosfetPin, int contPin, PyroManager& pyroManger)
    : _mosfetPin(mosfetPin), _contPin(contPin), _pyroManger(pyroManger)
{
    pinMode(_mosfetPin, OUTPUT);
    pinMode(_contPin, INPUT);
}

bool PyroChanel::hasContinuity() {
    return digitalRead(_contPin) == HIGH;
}

bool PyroChanel::fire(uint32_t duration) {
    if (!canFire()) return false;

    _fireTime = millis();
    _fireDuration = duration;
    digitalWrite(_mosfetPin, HIGH);
    return true;
}

void PyroChanel::stopFiring() {
    digitalWrite(_mosfetPin, LOW);
    _fireTime = 0;
    _fireDuration = 0;
}

void PyroChanel::update() {
    if (_fireTime > 0 && (millis() - _fireTime >= _fireDuration)) {
        stopFiring();
    }
}