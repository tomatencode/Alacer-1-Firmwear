#pragma once

#include <cstdint>

#include "PyroManager.hpp"

class PyroChannel {
public:
    PyroChannel(int mosfetPin, int contPin, PyroManager& pyroManger);

    void begin();

    bool canFire() { return hasContinuity() && _pyroManager.isArmed(); };
    bool fire(uint32_t duration);
    void stopFiring();

    bool hasContinuity();

    void update();
private:
    int _mosfetPin;
    int _contPin;

    uint32_t _fireTime = 0;
    uint32_t _fireDuration = 0;

    PyroManager& _pyroManager;
};