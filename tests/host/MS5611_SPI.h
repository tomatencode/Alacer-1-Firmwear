#pragma once
#include "SPI.h"
constexpr int MS5611_READ_OK = 0;
class MS5611_SPI {
public:
    inline static bool connected = true;
    inline static int result = 0;
    inline static float pressure = 101325;
    inline static float temperature = 20;
    MS5611_SPI(int, SPIClass*) {}
    bool begin() { return connected; }
    int read() { return result; }
    float getPressurePascal() const { return pressure; }
    float getTemperature() const { return temperature; }
};