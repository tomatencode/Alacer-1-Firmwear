#pragma once

class MotorIgniter{
public:
    MotorIgniter();

    bool canLight() { return true; };
    bool light() { return true; };

    bool didLight() { return true; };
};