#pragma once

#include "IMotorIgniter.hpp"
#include "../../hardwareIO/pyro/IPyroChannel.hpp"

namespace hardware {

class PyroMotorIgniter : public IMotorIgniter {
public:
    PyroMotorIgniter() = default;

    void setPyroChannel(IPyroChannel& pyroChannel) { _pyroChannel = &pyroChannel; };
    void clearPyroChannel() { _pyroChannel = nullptr; };

    bool canIgnite() override { return _pyroChannel && _pyroChannel->canFire(); };
    bool ignite() override { return _pyroChannel && _pyroChannel->fire(IGNITE_DURATION_ms); };
    bool stopIgniting() override { if (_pyroChannel)  { _pyroChannel->stopFiring(); return true; } return false; };
    
private:
    static constexpr uint32_t IGNITE_DURATION_ms = 1000;

    IPyroChannel* _pyroChannel = nullptr;
};

} // namespace hardware
