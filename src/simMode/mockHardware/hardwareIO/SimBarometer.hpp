#pragma once

#include "simMode/SimRocket.hpp"
#include "hardwareIO/barometer/Barometer.hpp"

namespace hardware {

class SimBarometer : public Barometer {
public:
    SimBarometer(SimRocket &simRocket);

    void begin() override;

    float getPressure_Pa() const override;
    float getTemperature_C() const override;

    void update() override;
private:
    SimRocket &_simRocket;
};

} // namespace hardware