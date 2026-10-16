#include "PyroChannel.hpp"

#include <Arduino.h>

namespace hardware {

PyroChannel::PyroChannel(const Clock& clock, int mosfetPin, int contPin, IPyroArmManager& pyroArmManager)
    : _clock(clock), _mosfetPin(mosfetPin), _contPin(contPin), _pyroArmManager(pyroArmManager)
{}

void PyroChannel::begin() {
    pinMode(_mosfetPin, OUTPUT);
    pinMode(_contPin, INPUT);
}

bool PyroChannel::hasContinuity() {
    return digitalRead(_contPin) == HIGH || _fireTime > 0; // assume continuity if currently firing
}

bool PyroChannel::fire(uint32_t duration) {
    if (!canFire()) return false;

    _fireTime = _clock.getTime_ms();
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
    bool expired = _fireTime > 0 && (_clock.getTime_ms() - _fireTime >= _fireDuration);
    bool unarmed = !_pyroArmManager.isArmed();

    if (firing && (expired || unarmed)) {
        stopFiring();
    }
}

} // namespace hardware
