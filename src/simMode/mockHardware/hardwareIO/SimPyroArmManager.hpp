#pragma once

#include "hardwareIO/pyro/IPyroArmManager.hpp"

namespace simHardware {

// Declarations only; hardware arming is supplied by the simulation harness.
class SimPyroArmManager : public hardware::IPyroArmManager {
public:
    SimPyroArmManager() = default;

    void begin() override {};
    bool isArmed() override { return _hardwareArm && _softwareArm; };
    bool isHardwareArmed() override { return _hardwareArm; };
    void softwareArm() override { _softwareArm = true; };
    void softwareDisarm() override { _softwareArm = false; };
    bool isSoftwareArmed() override { return _softwareArm; };

    void setHardwareArmed(bool armed) { _hardwareArm = armed; };

private:
    bool _hardwareArm = false;
    bool _softwareArm = false;
};

} // namespace simHardware