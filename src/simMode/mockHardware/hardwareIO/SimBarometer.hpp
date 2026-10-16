#pragma once

#include "simMode/SimRocket.hpp"
#include "hardwareIO/barometer/Barometer.hpp"

namespace hardware {

class SimBarometer : public Barometer {
public:
    SimBarometer(SimRocket &simRocket);

    void begin() override {};

    float getPressure_Pa() const {
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