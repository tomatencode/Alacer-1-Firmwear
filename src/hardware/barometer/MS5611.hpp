#pragma once

#include <SPI.h>
#include <MS5611_SPI.h>

#include "Barometer.hpp"

namespace hardware {

class MS5611 : public Barometer {
public:
    MS5611(int csPin, SPIClass &spi);

    void begin() override;

    float getAltitude() const override;
    float getPressure() const override;
    float getTemperature() const override;

    void update() override;
private:
    MS5611_SPI _sensor;

    float _altitude;
    float _pressure;
    float _temperature;
};

} // namespace hardware