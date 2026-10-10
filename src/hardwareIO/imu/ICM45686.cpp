#include <Arduino.h>
#include "ICM45686.hpp"

namespace {
    const SPISettings kSpiSettings(8000000, MSBFIRST, SPI_MODE0);
}

namespace hardware {

ICM45686::ICM45686(int csPin, SPIClass &spi)
    : _csPin(csPin), _spi(spi) {
}

void ICM45686::begin() {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    _spi.begin();

    uint8_t whoAmI = 0;
    readRegisters(REG_WHO_AM_I, &whoAmI, 1);
    _connected = (whoAmI == WHO_AM_I_VALUE);

    // ACCEL_UI_FS_SEL=001 (±16g), ACCEL_ODR=0111 (400Hz, LN mode)
    writeRegister(REG_ACCEL_CONFIG0, 0x17);
    // GYRO_UI_FS_SEL=0001 (±2000dps), GYRO_ODR=0111 (400Hz, LN mode)
    writeRegister(REG_GYRO_CONFIG0, 0x17);
    // GYRO_MODE=11, ACCEL_MODE=11 (both Low Noise)
    writeRegister(REG_PWR_MGMT0, 0x0F);

    delay(50); // gyro start-up time (max 35ms per datasheet) plus margin
}

void ICM45686::update() {
    uint8_t buffer[12];
    readRegisters(REG_ACCEL_DATA_X1_UI, buffer, sizeof(buffer));

    // Device default data endianness is little-endian: low address holds the LSB
    auto toInt16 = [](uint8_t lsb, uint8_t msb) -> int16_t {
        return static_cast<int16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
    };

    _accel.x_m_s2 = toInt16(buffer[0], buffer[1]) / ACCEL_LSB_PER_M_S2;
    _accel.y_m_s2 = toInt16(buffer[2], buffer[3]) / ACCEL_LSB_PER_M_S2;
    _accel.z_m_s2 = toInt16(buffer[4], buffer[5]) / ACCEL_LSB_PER_M_S2;

    _gyro.x_rad_s = toInt16(buffer[6], buffer[7]) / GYRO_LSB_PER_RAD_S;
    _gyro.y_rad_s = toInt16(buffer[8], buffer[9]) / GYRO_LSB_PER_RAD_S;
    _gyro.z_rad_s = toInt16(buffer[10], buffer[11]) / GYRO_LSB_PER_RAD_S;
}

IMU::Accel ICM45686::getAccel() const {
    return _accel;
}

IMU::Gyro ICM45686::getGyro() const {
    return _gyro;
}

bool ICM45686::isConnected() const {
    return _connected;
}

void ICM45686::writeRegister(uint8_t reg, uint8_t value) {
    _spi.beginTransaction(kSpiSettings);
    digitalWrite(_csPin, LOW);
    _spi.transfer(reg & 0x7F);
    _spi.transfer(value);
    digitalWrite(_csPin, HIGH);
    _spi.endTransaction();
}

void ICM45686::readRegisters(uint8_t reg, uint8_t *buffer, size_t length) {
    _spi.beginTransaction(kSpiSettings);
    digitalWrite(_csPin, LOW);
    _spi.transfer(reg | 0x80);
    for (size_t i = 0; i < length; ++i) {
        buffer[i] = _spi.transfer(0x00);
    }
    digitalWrite(_csPin, HIGH);
    _spi.endTransaction();
}

} // namespace hardware
