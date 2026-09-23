#include "MS5611.hpp"

namespace hardware {

MS5611::MS5611(int csPin, SPIClass &spi)
    : _sensor(csPin, &spi),  _pressure_Pa(0), _temperature_C(0) {}

void MS5611::begin() {
    _sensor.begin();
}

float MS5611::getPressure_Pa() const {
    return _pressure_Pa;
}

float MS5611::getTemperature_C() const {
    return _temperature_C;
}

void MS5611::update() {
    if (_sensor.read() != MS5611_READ_OK) {
        return;
    }

    _pressure_Pa = _sensor.getPressurePascal();
    _temperature_C = _sensor.getTemperature();
}

} // namespace hardware