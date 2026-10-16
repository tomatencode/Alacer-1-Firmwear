#pragma once

#include "IParachute.hpp"
#include "../../hardwareIO/pyro/IPyroChannel.hpp"

namespace hardware {

class Parachute : public IParachute {
public:
    Parachute() = default;

    void setPyroChannel(IPyroChannel& pyroChannel) { _pyroChannel = &pyroChannel; };
    void clearPyroChannel() { _pyroChannel = nullptr; };

    bool canDeploy() override { return _pyroChannel && _pyroChannel->canFire(); };
    bool deploy() override {
        if(_pyroChannel && _pyroChannel->fire(DEPLOY_PYRO_FIRE_DURATION_ms)) {
            _didDeploy = true;
            return true;
        }
        return false;
    };
    bool stopDeploying() override { if (_pyroChannel)  { _pyroChannel->stopFiring(); return true; } return false; };

    void reset() override { _didDeploy = false; };
    bool isDeployed() override { return _didDeploy; };
private:
    static constexpr uint32_t DEPLOY_PYRO_FIRE_DURATION_ms = 1000;

    IPyroChannel* _pyroChannel = nullptr;
    bool _didDeploy = false;
};

} // namespace hardware
