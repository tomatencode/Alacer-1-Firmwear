#pragma once

#include "../../hardwareIO/pyro/PyroChanel.hpp"

class Parashoot {
public:
    Parashoot() = default;

    void setPyroChanel(PyroChanel& pyroChanel) { _pyroChanel = &pyroChanel; };
    void clearPyroChanel() { _pyroChanel = nullptr; };

    bool canDeploy() { return _pyroChanel && _pyroChanel->canFire(); };
    bool deploy() {
        if(_pyroChanel && _pyroChanel->fire(1000)) {
            _didDeploy = true;
            return true;
        }
        return false;
    };

    void reset() { _didDeploy = false; };
    bool isDeployed() { return _didDeploy; };
private:
    PyroChanel* _pyroChanel = nullptr;
    bool _didDeploy = false;
};
