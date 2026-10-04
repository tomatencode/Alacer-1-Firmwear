#pragma once

#include <span>
#include <optional>

class PyroChannel;

class PyroManager {
public:
    PyroManager(int hardwareArmPin, std::span<PyroChannel*> pyroChannels);

    void begin();

    std::optional<PyroChannel*> getPyroChannel(size_t index);
    std::span<PyroChannel*> getPyroChannels();

    bool isArmed();

    bool isHardwareArmed();

    void softwareArm();
    void softwareDisarm();
    
    bool isSoftwareArmed();
private:
    int _hardwareArmPin;
    bool _softwareArm = false;

    std::span<PyroChannel*> _pyroChannels;
};