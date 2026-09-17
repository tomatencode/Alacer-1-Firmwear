#pragma once

class IMU {
public:
    struct Vector3 {
        float x;
        float y;
        float z;
    };

    virtual void begin() = 0;
    virtual void update() = 0;

    // Acceleration in g
    virtual Vector3 getAccel() const = 0;
    // Angular rate in degrees per second
    virtual Vector3 getGyro() const = 0;
};
