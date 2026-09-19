#include <Arduino.h>
#include <array>
#include <span>

#include <etl/vector.h>

#include "./hardware/radio/HC12.hpp"
#include "./hardware/led/BlinkLed.hpp"
#include "./hardware/buzzer/Buzzer.hpp"
#include "./hardware/imu/ICM45686.hpp"
#include "./hardware/barometer/MS5611.hpp"
#include "./hardware/Servo/Servo.hpp"
#include "./hardware/gimbal/Gimbal.hpp"

#include "./radioLink/Protocol.hpp"
#include "./radioLink/MessageScheduler.hpp"
#include "./radioLink/RequestHandlers.hpp"

hardware::BlinkLed statusLed(PB14, 35, 50);
hardware::Buzzer buzzer(PA8);

hardware::Servo pitchServo(PA1, 2, 0.0f, 180.0f, 90.0f, 544, 2400);
hardware::Servo yawServo(PA2, 3, 0.0f, 180.0f, 90.0f, 544, 2400);

hardware::Gimbal gimbal(
    pitchServo, yawServo,
    hardware::Gimbal::GimbalPos{0.0f, 0.0f},
    -10.0f, 10.0f,
    -10.0f, 10.0f,
    (10.0f/40.0f), (15.0f/80.0f),
    115.0f, 75.0f
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

    requestHandlers::attachDoBeep(messageScheduler, &buzzer);
    requestHandlers::attachGetImu(messageScheduler, &imu);
    requestHandlers::attachGetBarometer(messageScheduler, &barometer);
    requestHandlers::attachGetGimbal(messageScheduler, &gimbal);
    requestHandlers::attachSetGimbal(messageScheduler, &gimbal);

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