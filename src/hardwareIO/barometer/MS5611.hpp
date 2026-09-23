#pragma once

#include <SPI.h>
#include <MS5611_SPI.h>

#include "Barometer.hpp"

namespace hardware {

class MS5611 : public Barometer {
public:
    MS5611(int csPin, SPIClass &spi);

    void begin() override;

    float getPressure_Pa() const override;
    float getTemperature_C() const override;

    void update() override;
private:
    MS5611_SPI _sensor;
    
    float _pressure_Pa;
    float _temperature_C;
};

} // namespace hardware