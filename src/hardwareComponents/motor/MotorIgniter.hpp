#pragma once

#include "../../hardwareIO/pyro/PyroChanel.hpp"

class MotorIgniter{
public:
    MotorIgniter();

    void setPyroChanel(PyroChanel& pyroChanel) { _pyroChanel = &pyroChanel; };
    void clearPyroChanel() { _pyroChanel = nullptr; };

    bool canLight() { return _pyroChanel && _pyroChanel->canFire(); };
    bool light() { return _pyroChanel && _pyroChanel->fire(1000); };
private:
    PyroChanel* _pyroChanel = nullptr;
};