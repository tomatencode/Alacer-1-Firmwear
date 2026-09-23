#include "PyroManager.hpp"

#include <Arduino.h>

PyroManager::PyroManager(int hardwearArmPin)
    : _hardwareArmPin(hardwearArmPin), _softwearArm(false)
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

void PyroManager::SoftwareArm() {
    _softwearArm = true;
}

void PyroManager::SoftwareDisarm() {
    _softwearArm = false;
}

bool PyroManager::isSoftwareArmed() {
    return _softwearArm;
}