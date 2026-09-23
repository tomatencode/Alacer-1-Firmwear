#pragma once

#include "../../hardwareIO/pyro/PyroChannel.hpp"

class Parachute {
public:
    Parachute() = default;

    void setPyroChannel(PyroChannel& pyroChanel) { _pyroChanel = &pyroChanel; };
    void clearPyroChannel() { _pyroChanel = nullptr; };

    bool canDeploy() { return _pyroChanel && _pyroChanel->canFire(); };
    bool deploy() {
        if(_pyroChanel && _pyroChanel->fire(DEPLOY_PYRO_FIRE_DURATION_ms)) {
            _didDeploy = true;
            return true;
        }
        return false;
    };

    void reset() { _didDeploy = false; };
    bool isDeployed() { return _didDeploy; };
private:
    static constexpr uint32_t DEPLOY_PYRO_FIRE_DURATION_ms = 1000;

    PyroChannel* _pyroChanel = nullptr;
    bool _didDeploy = false;
};
