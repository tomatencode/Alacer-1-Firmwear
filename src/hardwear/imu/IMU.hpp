#pragma once

class IMU {
public:
    struct Accel {
        float x_m_s2;
        float y_m_s2;
        float z_m_s2;
    };

    struct Gyro {
        float x_rad_s;
        float y_rad_s;
        float z_rad_s;
    };

    virtual void begin() = 0;
    virtual void update() = 0;

    // Acceleration in m/s²
    virtual Accel getAccel() const = 0;
    // Angular rate in radians per second
    virtual Gyro getGyro() const = 0;
};
