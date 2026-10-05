#pragma once

#include <cstdint>

#include "PyroArmManager.hpp"

class PyroChannel {
public:
    PyroChannel(int mosfetPin, int contPin, PyroArmManager& pyroArmManager);

    void begin();

    bool canFire() { return hasContinuity() && _pyroArmManager.isArmed(); };
    bool fire(uint32_t duration);
    void stopFiring();

    bool hasContinuity();

    void update();
private:


    int _mosfetPin;
    int _contPin;

    uint32_t _fireTime = 0;
    uint32_t _fireDuration = 0;

    PyroArmManager& _pyroArmManager;
};