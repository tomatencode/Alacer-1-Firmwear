#pragma once

namespace hardware {

class Barometer {
public:
    virtual ~Barometer() = default;

    virtual void begin() = 0;

    virtual void update() = 0;

    virtual float getAltitude() const = 0;
    virtual float getPressure() const = 0;
    virtual float getTemperature() const = 0;
};

} // namespace hardware