#include <Arduino.h>
#include <array>
#include <span>

#include <etl/vector.h>

#include "./hardwear/radio/HC12.hpp"
#include "./hardwear/led/BlinkLed.hpp"
#include "./hardwear/buzzer/Buzzer.hpp"
#include "./hardwear/imu/ICM45686.hpp"
#include "./hardwear/barometer/MS5611.hpp"
#include "./hardwear/Servo/Servo.hpp"
#include "./hardwear/gimbal/Gimbal.hpp"

#include "./radioLink/Protocol.hpp"
#include "./radioLink/MessageSceduler.hpp"

hardware::BlinkLed statusLed(PB14, 35, 50);
hardware::Buzzer buzzer(PA8);

hardware::Servo pitchServo(PA1, 2, 0.0f, 180.0f, 90.0f, 544, 2400);
hardware::Servo yawServo(PA2, 3, 0.0f, 180.0f, 90.0f, 544, 2400);

hardware::Gimbal gimbal(
    pitchServo, yawServo,
    hardware::Gimbal::GimbalPos{0.0f, 0.0f},
    -10.0f, 10.0f,
    -10.0f, 10.0f,
    0.2f, 0.2f,
    -90.0f, -90.0f
);

hardware::HC12 radioHC12(PB15, PA9, PA10);

SPIClass sensorSPI(PA7, PA6, PA5);
hardware::ICM45686 imu(PB10, sensorSPI);
hardware::MS5611 barometer(PB3, sensorSPI);

Protocol::Parser radiolinkParser;
MessageScheduler messageScheduler(radiolinkParser, radioHC12);

const hardware::Buzzer::Melody startupMelody = {
    {262, 200},
    {294, 200},
    {330, 200}
};

void setup() {
    statusLed.begin();
    buzzer.begin();

    radioHC12.begin();

    imu.begin();
    barometer.begin();

    pitchServo.begin();
    yawServo.begin();
    gimbal.begin();

    messageScheduler.registerRequestHandler(
        Protocol::MessageType::DO_BEEP,
        [](std::span<const uint8_t>, MessageScheduler::HandlerResult resultCallback) {
            buzzer.beep(1000, 200);
            resultCallback(MessageScheduler::HandlerResultStatus::SUCCESS, {});
        });

    messageScheduler.registerRequestHandler(
        Protocol::MessageType::GET_IMU,
        [](std::span<const uint8_t>, MessageScheduler::HandlerResult resultCallback) {
            hardware::ICM45686::Accel accel = imu.getAccel();
            hardware::ICM45686::Gyro gyro = imu.getGyro();
            uint8_t payload[12];
            auto toBytes = [](int16_t value, uint8_t *buffer) {
                buffer[0] = value & 0xFF;
                buffer[1] = (value >> 8) & 0xFF;
            };
            toBytes(static_cast<int16_t>(accel.x_m_s2 * 1000), &payload[0]);
            toBytes(static_cast<int16_t>(accel.y_m_s2 * 1000), &payload[2]);
            toBytes(static_cast<int16_t>(accel.z_m_s2 * 1000), &payload[4]);
            toBytes(static_cast<int16_t>(gyro.x_rad_s * 1000), &payload[6]);
            toBytes(static_cast<int16_t>(gyro.y_rad_s * 1000), &payload[8]);
            toBytes(static_cast<int16_t>(gyro.z_rad_s * 1000), &payload[10]);
            resultCallback(MessageScheduler::HandlerResultStatus::SUCCESS, payload);
        });

    messageScheduler.registerRequestHandler(
        Protocol::MessageType::GET_BAROMETER,
        [](std::span<const uint8_t>, MessageScheduler::HandlerResult resultCallback) {
            float altitude = barometer.getAltitude();
            float pressure = barometer.getPressure();
            float temperature = barometer.getTemperature();
            uint8_t payload[12];
            auto toBytes = [](float value, uint8_t *buffer) {
                int32_t scaled = static_cast<int32_t>(value * 100);
                buffer[0] = scaled & 0xFF;
                buffer[1] = (scaled >> 8) & 0xFF;
                buffer[2] = (scaled >> 16) & 0xFF;
                buffer[3] = (scaled >> 24) & 0xFF;
            };
            toBytes(altitude, &payload[0]);
            toBytes(pressure, &payload[4]);
            toBytes(temperature, &payload[8]);
            resultCallback(MessageScheduler::HandlerResultStatus::SUCCESS, payload);
        });
    
    messageScheduler.registerRequestHandler(
        Protocol::MessageType::GET_GIMBAL,
        [](std::span<const uint8_t>, MessageScheduler::HandlerResult resultCallback) {
            hardware::Gimbal::GimbalPos gimbalPos = gimbal.getCurrentPos();
            uint8_t payload[8];
            auto toBytes = [](float value, uint8_t *buffer) {
                int16_t scaled = static_cast<int16_t>(value * 100);
                buffer[0] = scaled & 0xFF;
                buffer[1] = (scaled >> 8) & 0xFF;
            };
            toBytes(gimbalPos.pitch_deg, &payload[0]);
            toBytes(gimbalPos.yaw_deg, &payload[2]);
            resultCallback(MessageScheduler::HandlerResultStatus::SUCCESS, payload);
        });

    messageScheduler.registerRequestHandler(
        Protocol::MessageType::SET_GIMBAL,
        [](std::span<const uint8_t> payload, MessageScheduler::HandlerResult resultCallback) {
            if (payload.size() < 4) {
                resultCallback(MessageScheduler::HandlerResultStatus::FAILURE, {});
                return;
            }

            auto fromBytes = [](const uint8_t *buffer) {
                return static_cast<float>(static_cast<int16_t>(buffer[0] | (buffer[1] << 8))) / 100.0f;
            };
            hardware::Gimbal::GimbalPos targetPos;
            targetPos.pitch_deg = fromBytes(&payload[0]);
            targetPos.yaw_deg = fromBytes(&payload[2]);
            gimbal.setTarget(targetPos);
            resultCallback(MessageScheduler::HandlerResultStatus::SUCCESS, {});
        });

    buzzer.playMelody(startupMelody);
}

void loop() {
    buzzer.update();
    statusLed.update();
    radioHC12.update();
    imu.update();
    barometer.update();
    messageScheduler.update();
}