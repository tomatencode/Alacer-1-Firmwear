#pragma once

#include <span>
#include <optional>


namespace hardware {

class PyroArmManager {
public:
    PyroArmManager(int hardwareArmPin);

    void begin();

    bool isArmed();

    bool isHardwareArmed();

    void softwareArm();
    void softwareDisarm();
    
    bool isSoftwareArmed();
private:
    int _hardwareArmPin;
    bool _softwareArm = false;
};

} // namespace hardware
