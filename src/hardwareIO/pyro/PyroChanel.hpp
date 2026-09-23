#pragma once

#include <cstdint>

#include "PyroManager.hpp"

class PyroChanel {
public:
    PyroChanel(int mosfetPin, int contPin, PyroManager& pyroManger);

    bool canFire() { return hasContinuity() && _pyroManger.isArmed(); };
    bool fire(uint32_t duration);
    void stopFiring();

    bool hasContinuity();

    void update();
private:
    int _mosfetPin;
    int _contPin;

    uint32_t _fireTime;
    uint32_t _fireDuration;

    PyroManager& _pyroManger;
};