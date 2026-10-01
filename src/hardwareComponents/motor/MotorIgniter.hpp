#pragma once

#include "../../hardwareIO/pyro/PyroChannel.hpp"

class MotorIgniter{
public:
    MotorIgniter() = default;

    void setPyroChannel(PyroChannel& pyroChannel) { _pyroChannel = &pyroChannel; };
    void clearPyroChannel() { _pyroChannel = nullptr; };

    bool canIgnite() { return _pyroChannel && _pyroChannel->canFire(); };
    bool ignite() { return _pyroChannel && _pyroChannel->fire(IGNITE_DURATION_ms); };
private:
    static constexpr uint32_t IGNITE_DURATION_ms = 1000;

    PyroChannel* _pyroChannel = nullptr;
};