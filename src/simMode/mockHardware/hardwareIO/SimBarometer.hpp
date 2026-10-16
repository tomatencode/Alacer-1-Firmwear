#pragma once
#include <numbers>

#include "simMode/SimRocket.hpp"
#include "hardwareIO/barometer/Barometer.hpp"

namespace hardware {

class SimBarometer : public Barometer {
public:
    explicit SimBarometer(SimRocket& simRocket) : _simRocket(simRocket) {}

    void begin() override {};

    float getPressure_Pa() const override {
        const float altitude = _simRocket.getPosition().z();
        if (!std::isfinite(altitude)) {
            return _lastValidPressure_Pa;
        }

        const float noise_Pa = 0.4 * std::sin(static_cast<float>(std::rand()) / RAND_MAX * 2.0f * std::numbers::pi_v<float>) * 10.0f; // Add some noise if desired
        const float pressure_Pa = seaLevelPressure_Pa * std::exp(-altitude / scaleHeight_m) + noise_Pa;
        // Preserve the last good reading if the simulation or exponential is invalid.
        if (std::isfinite(pressure_Pa) && pressure_Pa > 0.0f) {
            _lastValidPressure_Pa = pressure_Pa;
        }
        return _lastValidPressure_Pa;
    };
    float getTemperature_C() const override {
        return 23.0f;
    };

    void update() override {};
private:
    static constexpr float seaLevelPressure_Pa = 101325.0f;
    static constexpr float scaleHeight_m = 8434.5f;

    SimRocket &_simRocket;
    mutable float _lastValidPressure_Pa = seaLevelPressure_Pa;
};

} // namespace hardware