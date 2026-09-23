#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <cstdint>
#include <cstddef>

#include "./IMU.hpp"

namespace hardware {

class ICM45686 : public IMU {
public:
    explicit ICM45686(int csPin, SPIClass &spi = SPI);

    void begin() override;
    void update() override;

    IMU::Accel getAccel() const override;
    IMU::Gyro getGyro() const override;

    // True if WHO_AM_I matched the expected value on begin()
    bool isConnected() const;

private:
    static constexpr uint8_t REG_ACCEL_DATA_X1_UI = 0x00;
    static constexpr uint8_t REG_PWR_MGMT0        = 0x10;
    static constexpr uint8_t REG_ACCEL_CONFIG0    = 0x1B;
    static constexpr uint8_t REG_GYRO_CONFIG0     = 0x1C;
    static constexpr uint8_t REG_WHO_AM_I         = 0x72;
    static constexpr uint8_t WHO_AM_I_VALUE       = 0xE9;

    // Configured full-scale ranges: accel ±16g, gyro ±2000dps
    static constexpr float ACCEL_LSB_PER_M_S2 = 208.84f;
    static constexpr float GYRO_LSB_PER_RAD_S = 939.65f;

    void writeRegister(uint8_t reg, uint8_t value);
    void readRegisters(uint8_t reg, uint8_t *buffer, size_t length);

    int _csPin;
    SPIClass &_spi;

    bool _connected = false;

    IMU::Accel _accel{0, 0, 0};
    IMU::Gyro _gyro{0, 0, 0};
};

} // namespace hardware
