#pragma once

class PyroManager {
public:
    PyroManager(int hardwareArmPin);

    void begin();

    bool isArmed();

    bool isHardwareArmed();

    void SoftwareArm();
    void SoftwareDisarm();
    
    bool isSoftwareArmed();
private:
    int _hardwareArmPin;
    bool _softwearArm = false;
};