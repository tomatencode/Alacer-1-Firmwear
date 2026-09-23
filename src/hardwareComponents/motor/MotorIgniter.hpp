#pragma once

#include "../../hardwareIO/pyro/PyroChannel.hpp"

class MotorIgniter{
public:
    MotorIgniter() = default;

    void setPyroChannel(PyroChannel& pyroChanel) { _pyroChanel = &pyroChanel; };
    void clearPyroChannel() { _pyroChanel = nullptr; };

    bool canIgnite() { return _pyroChanel && _pyroChanel->canFire(); };
    bool ignite() { return _pyroChanel && _pyroChanel->fire(IGNITE_DURATION_ms); };
private:
    static constexpr uint32_t IGNITE_DURATION_ms = 1000;

    PyroChannel* _pyroChanel = nullptr;
};