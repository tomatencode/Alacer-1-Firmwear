#include "RequestHandlers.hpp"

namespace {

hardware::Buzzer* gBuzzer = nullptr;
hardware::ICM45686* gImu = nullptr;
hardware::Barometer* gBarometer = nullptr;
hardware::Gimbal* gGimbal = nullptr;

void doBeepRequestHandler(std::span<const uint8_t>, MessageScheduler::HandlerResult resultCallback) {
    if (gBuzzer == nullptr) {
        resultCallback(MessageScheduler::HandlerResultStatus::FAILURE, {});
        return;
    }

    gBuzzer->beep(1000, 200);
    resultCallback(MessageScheduler::HandlerResultStatus::SUCCESS, {});
}

void getImuRequestHandler(std::span<const uint8_t>, MessageScheduler::HandlerResult resultCallback) {
    if (gImu == nullptr) {
        resultCallback(MessageScheduler::HandlerResultStatus::FAILURE, {});
        return;
    }

    hardware::ICM45686::Accel accel = gImu->getAccel();
    hardware::ICM45686::Gyro gyro = gImu->getGyro();
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
}

void getBarometerRequestHandler(std::span<const uint8_t>, MessageScheduler::HandlerResult resultCallback) {
    if (gBarometer == nullptr) {
        resultCallback(MessageScheduler::HandlerResultStatus::FAILURE, {});
        return;
    }

    float altitude = gBarometer->getAltitude();
    float pressure = gBarometer->getPressure();
    float temperature = gBarometer->getTemperature();
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
}

void getGimbalRequestHandler(std::span<const uint8_t>, MessageScheduler::HandlerResult resultCallback) {
    if (gGimbal == nullptr) {
        resultCallback(MessageScheduler::HandlerResultStatus::FAILURE, {});
        return;
    }

    hardware::Gimbal::GimbalPos gimbalPos = gGimbal->getCurrentPos();
    uint8_t payload[8];
    auto toBytes = [](float value, uint8_t *buffer) {
        int16_t scaled = static_cast<int16_t>(value * 100);
        buffer[0] = scaled & 0xFF;
        buffer[1] = (scaled >> 8) & 0xFF;
    };
    toBytes(gimbalPos.pitch_deg, &payload[0]);
    toBytes(gimbalPos.yaw_deg, &payload[2]);
    resultCallback(MessageScheduler::HandlerResultStatus::SUCCESS, payload);
}

void setGimbalRequestHandler(std::span<const uint8_t> payload, MessageScheduler::HandlerResult resultCallback) {
    if (gGimbal == nullptr) {
        resultCallback(MessageScheduler::HandlerResultStatus::FAILURE, {});
        return;
    }

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
    gGimbal->setTarget(targetPos);
    resultCallback(MessageScheduler::HandlerResultStatus::SUCCESS, {});
}

} // namespace

namespace requestHandlers {

void attachDoBeep(MessageScheduler& messageScheduler, hardware::Buzzer* buzzer) {
    gBuzzer = buzzer;
    messageScheduler.registerRequestHandler(
        Protocol::MessageType::DO_BEEP,
        MessageScheduler::Handler::create(doBeepRequestHandler));
}

void attachGetImu(MessageScheduler& messageScheduler, hardware::ICM45686* imu) {
    gImu = imu;
    messageScheduler.registerRequestHandler(
        Protocol::MessageType::GET_IMU,
        MessageScheduler::Handler::create(getImuRequestHandler));
}

void attachGetBarometer(MessageScheduler& messageScheduler, hardware::Barometer* barometer) {
    gBarometer = barometer;
    messageScheduler.registerRequestHandler(
        Protocol::MessageType::GET_BAROMETER,
        MessageScheduler::Handler::create(getBarometerRequestHandler));
}

void attachGetGimbal(MessageScheduler& messageScheduler, hardware::Gimbal* gimbal) {
    gGimbal = gimbal;
    messageScheduler.registerRequestHandler(
        Protocol::MessageType::GET_GIMBAL,
        MessageScheduler::Handler::create(getGimbalRequestHandler));
}

void attachSetGimbal(MessageScheduler& messageScheduler, hardware::Gimbal* gimbal) {
    gGimbal = gimbal;
    messageScheduler.registerRequestHandler(
        Protocol::MessageType::SET_GIMBAL,
        MessageScheduler::Handler::create(setGimbalRequestHandler));
}

} // namespace requestHandlers