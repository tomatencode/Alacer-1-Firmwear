#include "PyroArmManager.hpp"

#include <Arduino.h>

#include "PyroChannel.hpp"

PyroArmManager::PyroArmManager(int hardwareArmPin)
    : _hardwareArmPin(hardwareArmPin), _softwareArm(false)
{}

void PyroArmManager::begin() {
    pinMode(_hardwareArmPin, INPUT);
}

bool PyroArmManager::isArmed() {
    return isHardwareArmed() && isSoftwareArmed();
}

bool PyroArmManager::isHardwareArmed() {
    return digitalRead(_hardwareArmPin) == HIGH;
}

void PyroArmManager::softwareArm() {
    _softwareArm = true;
}

void PyroArmManager::softwareDisarm() {
    _softwareArm = false;
}

bool PyroArmManager::isSoftwareArmed() {
    return _softwareArm;
}
