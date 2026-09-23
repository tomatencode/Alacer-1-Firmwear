#include "PyroChannel.hpp"

#include <Arduino.h>

PyroChannel::PyroChannel(int mosfetPin, int contPin, PyroManager& pyroManger)
    : _mosfetPin(mosfetPin), _contPin(contPin), _pyroManager(pyroManger)
{}

void PyroChannel::begin() {
    pinMode(_mosfetPin, OUTPUT);
    pinMode(_contPin, INPUT);
}

bool PyroChannel::hasContinuity() {
    return digitalRead(_contPin) == HIGH;
}

bool PyroChannel::fire(uint32_t duration) {
    if (!canFire()) return false;

    _fireTime = millis();
    _fireDuration = duration;
    digitalWrite(_mosfetPin, HIGH);
    return true;
}

void PyroChannel::stopFiring() {
    digitalWrite(_mosfetPin, LOW);
    _fireTime = 0;
    _fireDuration = 0;
}

void PyroChannel::update() {
    bool firing = _fireTime > 0;
    bool expired = _fireTime > 0 && (millis() - _fireTime >= _fireDuration);
    bool unArmed = !_pyroManager.isArmed();

    if (firing && (expired || unArmed)) {
        stopFiring();
    }
}