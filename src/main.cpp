#include <Arduino.h>
#include <array>
#include <span>
#include "./hardwear/imu/ICM45686.hpp"

#include <etl/vector.h>

#include "./hardwear/radio/HC12.hpp"
#include "./hardwear/led/BlinkLed.hpp"
#include "./hardwear/buzzer/Buzzer.hpp"

#include "./radioLink/Protocol.hpp"
#include "./radioLink/MessageSceduler.hpp"


Buzzer buzzer(PA8);
BlinkLed statusLed(PB14, 35, 50);
HC12 radioHC12(PB15, PA9, PA10);

SPIClass sensorSPI(PA7, PA6, PA5);
ICM45686 imu(PB10, sensorSPI);

Protocol::Parser radiolinkParser;
MessageScheduler messageScheduler(radiolinkParser, radioHC12);

const Buzzer::Melody startupMelody = {
    {262, 200},
    {294, 200},
    {330, 200}
};

void setup() {
    statusLed.begin();
    radioHC12.begin();
    buzzer.begin();
    imu.begin();

    buzzer.playMelody(startupMelody);

    radioHC12.sendATCommand("AT+DEFAULT");

    messageScheduler.registerRequestHandler(
        Protocol::MessageType::DO_BEEP,
        [](std::span<const uint8_t>, MessageScheduler::HandlerResult resultCallback) {
            buzzer.beep(1000, 200);
            resultCallback(MessageScheduler::HandlerResultStatus::SUCCESS, {});
        });

    messageScheduler.registerRequestHandler(
        Protocol::MessageType::GET_IMU,
        [](std::span<const uint8_t>, MessageScheduler::HandlerResult resultCallback) {
            IMU::Vector3 accel = imu.getAccel();
            IMU::Vector3 gyro = imu.getGyro();
            uint8_t payload[12];
            auto toBytes = [](int16_t value, uint8_t *buffer) {
                buffer[0] = value & 0xFF;
                buffer[1] = (value >> 8) & 0xFF;
            };
            toBytes(static_cast<int16_t>(accel.x * 1000), &payload[0]);
            toBytes(static_cast<int16_t>(accel.y * 1000), &payload[2]);
            toBytes(static_cast<int16_t>(accel.z * 1000), &payload[4]);
            toBytes(static_cast<int16_t>(gyro.x * 1000), &payload[6]);
            toBytes(static_cast<int16_t>(gyro.y * 1000), &payload[8]);
            toBytes(static_cast<int16_t>(gyro.z * 1000), &payload[10]);
            resultCallback(MessageScheduler::HandlerResultStatus::SUCCESS, payload);
        });

}

void loop() {
    buzzer.update();
    statusLed.update();
    radioHC12.update();
    messageScheduler.update();
    // imu.update();
}