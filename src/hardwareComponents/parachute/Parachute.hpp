#pragma once

#include "../../hardwareIO/pyro/PyroChannel.hpp"

class Parachute {
public:
    Parachute() = default;

    void setPyroChannel(PyroChannel& pyroChannel) { _pyroChannel = &pyroChannel; };
    void clearPyroChannel() { _pyroChannel = nullptr; };

    bool canDeploy() { return _pyroChannel && _pyroChannel->canFire(); };
    bool deploy() {
        if(_pyroChannel && _pyroChannel->fire(DEPLOY_PYRO_FIRE_DURATION_ms)) {
            _didDeploy = true;
            return true;
        }
        return false;
    };
    bool stopDeploying() { if (_pyroChannel)  { _pyroChannel->stopFiring(); return true; } return false; };

    void reset() { _didDeploy = false; };
    bool isDeployed() { return _didDeploy; };
private:
    static constexpr uint32_t DEPLOY_PYRO_FIRE_DURATION_ms = 1000;

    PyroChannel* _pyroChannel = nullptr;
    bool _didDeploy = false;
};
