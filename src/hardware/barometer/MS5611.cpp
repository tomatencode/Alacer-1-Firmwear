#include "MS5611.hpp"

namespace hardware {

MS5611::MS5611(int csPin, SPIClass &spi)
    : _sensor(csPin, &spi), _altitude(0), _pressure(0), _temperature(0) {}

void MS5611::begin() {
    _sensor.begin();
}

float MS5611::getAltitude() const {
    return _altitude;
}

float MS5611::getPressure() const {
    return _pressure;
}

float MS5611::getTemperature() const {
    return _temperature;
}

void MS5611::update() {
    if (_sensor.read() != MS5611_READ_OK) {
        return;
    }

    _pressure = _sensor.getPressure();
    _temperature = _sensor.getTemperature();
    _altitude = _sensor.getAltitude();
}

} // namespace hardware