#include "PyroManager.hpp"

#include <Arduino.h>

PyroManager::PyroManager(int hardwareArmPin)
    : _hardwareArmPin(hardwareArmPin), _softwareArm(false)
{}

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