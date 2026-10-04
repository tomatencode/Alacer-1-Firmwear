#include "PyroManager.hpp"

#include <Arduino.h>

#include "PyroChannel.hpp"

PyroManager::PyroManager(int hardwareArmPin, std::span<PyroChannel*> pyroChannels)
    : _hardwareArmPin(hardwareArmPin), _softwareArm(false), _pyroChannels(pyroChannels)
{
    for (auto& channel : _pyroChannels) {
        if (channel) {
            channel->setPyroManager(this);
        }
    }
}

void PyroManager::begin() {
    pinMode(_hardwareArmPin, INPUT);
}

bool PyroManager::isArmed() {
    return isHardwareArmed() && isSoftwareArmed();
}

bool PyroManager::isHardwareArmed() {
    return digitalRead(_hardwareArmPin) == HIGH;
}

void PyroManager::softwareArm() {
    _softwareArm = true;
}

void PyroManager::softwareDisarm() {
    _softwareArm = false;
}

bool PyroManager::isSoftwareArmed() {
    return _softwareArm;
}

std::optional<PyroChannel*> PyroManager::getPyroChannel(size_t index) {
    if (index < _pyroChannels.size()) {
        return _pyroChannels[index];
    } else {
        return std::nullopt;
    }
}

std::span<PyroChannel*> PyroManager::getPyroChannels() {
    return _pyroChannels;
}