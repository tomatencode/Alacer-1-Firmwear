#pragma once

class PyroManager {
public:
    PyroManager(int hardwearArmPin);

    bool isArmed();

    bool isHardwareArmed();

    void SoftwearArm();
    void SoftwearDisarm();
    
    bool isSoftwearArmed();
private:
    int _hardwearArmPin;
    bool _softwearArm = false;
};