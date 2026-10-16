#pragma once

namespace hardware {

class IPyroArmManager {
public:
    virtual ~IPyroArmManager() = default;

    virtual void begin() = 0;
    virtual bool isArmed() = 0;
    virtual bool isHardwareArmed() = 0;
    virtual void softwareArm() = 0;
    virtual void softwareDisarm() = 0;
    virtual bool isSoftwareArmed() = 0;
};

} // namespace hardware