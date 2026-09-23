#include "PyroManager.hpp"

#include <Arduino.h>

PyroManager::PyroManager(int hardwearArmPin)
    : _hardwearArmPin(hardwearArmPin), _softwearArm(false)
{
    pinMode(_hardwearArmPin, INPUT);
}

bool PyroManager::isArmed() {
    return isHardwareArmed() || isSoftwearArmed();
}

bool PyroManager::isHardwareArmed() {
    return digitalRead(_hardwearArmPin) == HIGH;
}

void PyroManager::SoftwearArm() {
    _softwearArm = true;
}

void PyroManager::SoftwearDisarm() {
    _softwearArm = false;
}

bool PyroManager::isSoftwearArmed() {
    return _softwearArm;
}