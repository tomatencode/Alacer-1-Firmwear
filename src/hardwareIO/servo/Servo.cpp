#include "Servo.hpp"


namespace hardware {

namespace {
HardwareTimer servoTimer(TIM2);
}

Servo::Servo(uint32_t pin, uint32_t channel, float minAngle_deg, float maxAngle_deg, float initialAngle_deg, int minPulse_us, int maxPulse_us, float servoSpeed_deg_s)
    : _pin(pin),
      _channel(channel),
      _minAngle_deg(minAngle_deg),
      _maxAngle_deg(maxAngle_deg),
      _targetAngle_deg(initialAngle_deg),
      _minPulse_us(minPulse_us),
      _maxPulse_us(maxPulse_us),
      _servoSpeed_deg_s(servoSpeed_deg_s),
      _targetSetTime(0),
      _targetSetPosition_deg(initialAngle_deg)
{}

void Servo::begin() {
    servoTimer.setPWM(
        _channel,
        _pin,
        50,
        angleToDuty(_targetAngle_deg)
    );
}

void Servo::setTarget(float angle_deg) {
    _targetAngle_deg = std::clamp(angle_deg, _minAngle_deg, _maxAngle_deg);

    servoTimer.setCaptureCompare(
        _channel,
        angleToDuty(_targetAngle_deg),
        PERCENT_COMPARE_FORMAT
    );

    _targetSetTime = millis();
    _targetSetPosition_deg = getPosition_deg();
}

float Servo::getTarget_deg() const {
    return _targetAngle_deg;
}

float Servo::getPosition_deg() const {
    const uint32_t elapsedTime = millis() - _targetSetTime;
    const float maxDelta = _servoSpeed_deg_s * (elapsedTime / 1000.0);
    const float delta = _targetAngle_deg - _targetSetPosition_deg;
    const float clampedDelta = std::clamp(delta, -maxDelta, maxDelta);

    return _targetSetPosition_deg + clampedDelta;
}

float Servo::angleToDuty(float angle_deg) const {
    const float pulseUs =
        _minPulse_us +
        (angle_deg - _minAngle_deg) *
        (_maxPulse_us - _minPulse_us) /
        (_maxAngle_deg - _minAngle_deg);

    return pulseUs / 20000.0f * 100.0f;
}

} // namespace hardware