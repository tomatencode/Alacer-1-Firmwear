#pragma once

namespace hardware {

class Barometer {
public:
    virtual ~Barometer() = default;

    virtual float getAltitude() const = 0;
    virtual float getPressure() const = 0;
    virtual float getTemperature() const = 0;
};

} // namespace hardware