#pragma once
#include <cmath>

#include "simMode/SimRocket.hpp"
#include "hardwareIO/barometer/Barometer.hpp"

namespace hardware {

class SimBarometer : public Barometer {
public:
    explicit SimBarometer(SimRocket& simRocket) : _simRocket(simRocket) {}

    void begin() override {};

    float getPressure_Pa() const override {
        float altitude = _simRocket.getPosition().z();
        return 101325.0f * std::exp(-altitude / 8434.5f);
    };
    float getTemperature_C() const override {
        return 23.0f;
    };

    void update() override {};
private:
    SimRocket &_simRocket;
};

} // namespace hardware