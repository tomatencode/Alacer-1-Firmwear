#pragma once

namespace hardware {

class Barometer {
public:
    virtual ~Barometer() = default;

    virtual void begin() = 0;

    virtual void update() = 0;

    virtual float getPressure_Pa() const = 0;
    virtual float getTemperature_C() const = 0;
};

} // namespace hardware