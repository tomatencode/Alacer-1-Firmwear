#pragma once

#include "IPyroArmManager.hpp"

namespace hardware {

class PyroArmManager : public IPyroArmManager {
public:
    PyroArmManager(int hardwareArmPin);

    void begin() override;

    bool isArmed() override;

    bool isHardwareArmed() override;

    void softwareArm() override;
    void softwareDisarm() override;
    
    bool isSoftwareArmed() override;
private:
    int _hardwareArmPin;
    bool _softwareArm = false;
};

} // namespace hardware
