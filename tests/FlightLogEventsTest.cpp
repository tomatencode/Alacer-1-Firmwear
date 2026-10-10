#include <algorithm>
#include <array>
#include <cassert>
#include <iostream>
#include <vector>

#include "logManagement/LogManager.hpp"
#include "hardwareIO/pyro/PyroChannel.hpp"
#include "radioLink/MessageScheduler.hpp"
#include "helpers/codec/FixedPointCodec.hpp"
#include "hardwareIO/imu/ICM45686.hpp"
#include "hardwareIO/barometer/MS5611.hpp"
#include "flightStateManagement/FlightStateManager.hpp"
#include "logManagement/TelemetryRecorder.hpp"
#include "hardwareIO/battery/Battery.hpp"
#include <limits>

// Software-only servo implementation for exercising the real gimbal/PID code.
hardware::Servo::Servo(uint32_t pin, uint32_t channel, float low, float high, float initial,
                       int minPulse, int maxPulse, float speed)
    : _pin(pin), _channel(channel), _targetAngle_deg(initial), _minAngle_deg(low),
      _maxAngle_deg(high), _minPulse_us(minPulse), _maxPulse_us(maxPulse),
      _servoSpeed_deg_s(speed), _targetSetTime(0), _targetSetPosition_deg(initial) {}
void hardware::Servo::begin() {}
void hardware::Servo::setTarget(float angle) { _targetAngle_deg = std::clamp(angle, _minAngle_deg, _maxAngle_deg); }
float hardware::Servo::getTarget_deg() const { return _targetAngle_deg; }
float hardware::Servo::getPosition_deg() const { return _targetAngle_deg; }

class MemoryFlash : public hardware::FlashChip {
public:
    bool failWrites = false;
    std::vector<uint8_t> bytes = std::vector<uint8_t>(1024 * 1024, 0xFF);
    void begin() override {}
    bool isConnected() const override { return true; }
    uint32_t getCapacity() const override { return bytes.size(); }
    size_t getPageSize() const override { return 256; }
    size_t getSectorSize() const override { return 4096; }
    bool read(uint32_t address, std::span<uint8_t> data) override {
        if (address > bytes.size() || data.size() > bytes.size() - address) return false;
        std::copy_n(bytes.begin() + address, data.size(), data.begin());
        return true;
    }
    bool write(uint32_t address, std::span<const uint8_t> data) override {
        if (failWrites) return false;
        if (address > bytes.size() || data.size() > bytes.size() - address) return false;
        for (size_t i = 0; i < data.size(); ++i) bytes[address + i] &= data[i];
        return true;
    }
    bool eraseSector(uint32_t address) override {
        if (address % 4096 || address > bytes.size() - 4096) return false;
        std::fill_n(bytes.begin() + address, 4096, 0xFF);
        return true;
    }
    bool eraseChip() override {
        std::fill(bytes.begin(), bytes.end(), 0xFF);
        return true;
    }
};

class TestRadio : public hardware::Radio {
public:
    std::vector<uint8_t> incoming;
    size_t cursor = 0;
    bool send(std::span<const uint8_t>) override { return false; } // keep response queue full
    bool available() override { return cursor < incoming.size(); }
    std::optional<uint8_t> read() override {
        if (!available()) return std::nullopt;
        return incoming[cursor++];
    }
    void enqueue(Protocol::MessageType type, uint8_t sequence) {
        Protocol::Frame frame{};
        frame.numMessages = 1;
        Protocol::Message message{};
        message.type = type;
        message.seqId = sequence;
        frame.messages.push_back(message);
        std::array<uint8_t, Protocol::MAX_FRAME_SIZE> buffer{};
        const auto size = Protocol::encode(frame, buffer);
        assert(size);
        incoming.insert(incoming.end(), buffer.begin(), buffer.begin() + *size);
    }
};

template<typename T>
std::array<uint8_t, LogProtocol::maxEventSize> encode(const T& event, LogProtocol::EventType type, size_t size) {
    std::array<uint8_t, LogProtocol::maxEventSize> bytes;
    bytes.fill(0xCC);
    assert(LogProtocol::encodeEvent(event, 0x1234, std::span(bytes).first(size)) == size);
    assert(bytes[0] == static_cast<uint8_t>(type));
    assert(bytes[1] == 0x34 && bytes[2] == 0x12);
    assert(bytes[size] == 0xCC); // encoder must not write past the record
    return bytes;
}

void testEncoding() {
    using namespace LogProtocol;
    auto bytes = encode(BarometricAltitudeEvent{-12.5f}, EventType::BarometricAltitude, 7);
    assert(fixedPoint::decode32(bytes, 3) == -12.5f);
    bytes = encode(GimbalPositionEvent{1.25f, -2.5f, 3.75f, -4.0f}, EventType::GimbalPosition, 19);
    assert(fixedPoint::decode32(bytes, 3) == 1.25f);
    assert(fixedPoint::decode32(bytes, 7) == -2.5f);
    assert(fixedPoint::decode32(bytes, 11) == 3.75f);
    assert(fixedPoint::decode32(bytes, 15) == -4.0f);
    bytes = encode(RocketRotationEvent{Eigen::Quaternionf(0.5f, -0.25f, 0.75f, 1.0f)}, EventType::RocketRotation, 19);
    assert(fixedPoint::decode32(bytes, 3) == -0.25f);
    assert(fixedPoint::decode32(bytes, 7) == 0.75f);
    assert(fixedPoint::decode32(bytes, 11) == 1.0f);
    assert(fixedPoint::decode32(bytes, 15) == 0.5f);
    bytes = encode(RocketPositionEvent{-10.5f, 20.25f, 100.0f}, EventType::RocketPosition, 15);
    assert(fixedPoint::decode32(bytes, 3) == -10.5f);
    assert(fixedPoint::decode32(bytes, 7) == 20.25f);
    assert(fixedPoint::decode32(bytes, 11) == 100.0f);
    bytes = encode(FlightStateTransitionEvent{1, 6}, EventType::FlightStateTransition, 5);
    assert(bytes[3] == 1 && bytes[4] == 6);
    bytes = encode(PyroFiringEvent{2, 0x12345678}, EventType::PyroFiring, 8);
    assert(bytes[3] == 2 && littleEndian::decodeU32(bytes, 4) == 0x12345678);
    bytes = encode(RadioMessageDroppedEvent{0xFE, 42, RadioDropReason::ResponseQueueFull}, EventType::RadioMessageDropped, 6);
    assert(bytes[3] == 0xFE && bytes[4] == 42 && bytes[5] == 1);
    bytes = encode(BatteryVoltageEvent{3.75f}, EventType::BatteryVoltage, 7);
    assert(fixedPoint::decode32(bytes, 3) == 3.75f);
    bytes = encode(PyroArmStateEvent{true, false}, EventType::PyroArmState, 5);
    assert(bytes[3] == 1 && bytes[4] == 0);
    bytes = encode(PyroContinuityEvent{2, true}, EventType::PyroContinuity, 5);
    assert(bytes[3] == 2 && bytes[4] == 1);
    bytes = encode(PyroFireRefusedEvent{1, 500, PyroRefusalReason::Both}, EventType::PyroFireRefused, 9);
    assert(bytes[3] == 1 && littleEndian::decodeU32(bytes, 4) == 500 && bytes[8] == 3);
    bytes = encode(PyroStoppedEvent{2, 250, PyroStopReason::Disarmed}, EventType::PyroStopped, 9);
    assert(bytes[3] == 2 && littleEndian::decodeU32(bytes, 4) == 250 && bytes[8] == 2);
    bytes = encode(PIDEvent{true, 1.25f, -2.5f, 3.75f, -4, 5, -6, true, false}, EventType::PID, 30);
    assert(bytes[3] == 1 && bytes[28] == 1 && bytes[29] == 0);
    const std::array<float, 6> pidValues{1.25f, -2.5f, 3.75f, -4, 5, -6};
    for (size_t i = 0; i < pidValues.size(); ++i) assert(fixedPoint::decode32(bytes, 4 + 4 * i) == pidValues[i]);
    bytes = encode(VelocityEvent{1, -2, 3, false, true}, EventType::Velocity, 17);
    assert(fixedPoint::decode32(bytes, 3) == 1 && fixedPoint::decode32(bytes, 7) == -2);
    assert(fixedPoint::decode32(bytes, 11) == 3 && bytes[15] == 0 && bytes[16] == 1);
    bytes = encode(FlightDetectionEvent{FlightDetection::Touchdown, 2, -0.5f}, EventType::FlightDetection, 12);
    assert(bytes[3] == 1 && fixedPoint::decode32(bytes, 4) == 2 && fixedPoint::decode32(bytes, 8) == -0.5f);
    bytes = encode(SensorHealthEvent{SensorId::Barometer, false, true, SensorError::Read}, EventType::SensorHealth, 7);
    assert(bytes[3] == 1 && bytes[4] == 0 && bytes[5] == 1 && bytes[6] == 3);
    bytes = encode(RadioDiagnosticsEvent{1, 2, 3, 4}, EventType::RadioDiagnostics, 19);
    for (size_t i = 0; i < 4; ++i) assert(littleEndian::decodeU32(bytes, 3 + 4 * i) == i + 1);
}

void testSourceEvents() {
    MemoryFlash flash;
    StorageManager storage(flash);
    storage.begin();
    LogManager logger(storage);
    const LogProtocol::LogMetadata metadata{0, Eigen::Quaternionf::Identity(), Eigen::Quaternionf::Identity(), 0, 0, 0, 0};
    const StorageManager::Filename filename("events.bin");
    assert(logger.startLog(metadata, filename));

    PyroArmManager arm(1);
    PyroChannel pyro(2, 3, arm, &logger, 2);
    pyro.begin();
    hostArduino::pins[1] = HIGH;
    hostArduino::pins[3] = HIGH;
    assert(!pyro.fire(500)); // software disarmed: refusal event
    arm.softwareArm();
    assert(pyro.fire(500));
    assert(hostArduino::pins[2] == HIGH);
    pyro.stopFiring();
    assert(pyro.fire(500)); // identical actions must each be recorded
    arm.softwareDisarm();
    pyro.update();
    assert(hostArduino::pins[2] == LOW);

    TestRadio radio;
    Protocol::Parser parser;
    MessageScheduler scheduler(parser, radio, {}, &logger);
    radio.enqueue(static_cast<Protocol::MessageType>(0xFE), 42);
    for (uint8_t i = 0; i < 34; ++i) radio.enqueue(Protocol::MessageType::PING, i);
    scheduler.update();
    assert(scheduler.getDroppedMessages() == 3); // unknown request + two queue overflows
    assert(logger.finishLog());

    std::array<uint8_t, 90> records{};
    assert(storage.readFile(filename, LogProtocol::headerSize, records) == records.size());
    assert(records[0] == static_cast<uint8_t>(LogProtocol::EventType::PyroFireRefused));
    assert(records[8] == 1);
    for (size_t offset : {size_t(9), size_t(26)}) {
        assert(records[offset] == static_cast<uint8_t>(LogProtocol::EventType::PyroFiring));
        assert(records[offset + 3] == 2);
        assert(littleEndian::decodeU32(records, offset + 4) == 500);
    }
    assert(records[17] == static_cast<uint8_t>(LogProtocol::EventType::PyroStopped) && records[25] == 0);
    assert(records[34] == static_cast<uint8_t>(LogProtocol::EventType::PyroStopped) && records[42] == 2);
    assert(records[43] == static_cast<uint8_t>(LogProtocol::EventType::PyroContinuity));
    assert(records[48] == static_cast<uint8_t>(LogProtocol::EventType::RadioMessageDropped));
    assert(records[51] == 0xFE && records[52] == 42 && records[53] == 0);
    for (size_t offset : {size_t(54), size_t(60)}) {
        assert(records[offset] == static_cast<uint8_t>(LogProtocol::EventType::RadioMessageDropped));
        assert(records[offset + 3] == 0 && records[offset + 5] == 1);
    }
    assert(records[58] == 32 && records[64] == 33);
    assert(records[66] == static_cast<uint8_t>(LogProtocol::EventType::RadioDiagnostics));
    assert(littleEndian::decodeU32(records, 81) == 1); // failed transmission
    assert(records[85] == static_cast<uint8_t>(LogProtocol::EventType::TimeSync));
    assert(!logger.appendEvent(LogProtocol::BarometricAltitudeEvent{123}));
}

const LogProtocol::LogMetadata testMetadata{
    0, Eigen::Quaternionf::Identity(), Eigen::Quaternionf::Identity(), 0, 0, 0, 0};

template<typename T>
void testCache(const T& first, const T& changed) {
    LastEventsTracker tracker;
    assert(!tracker.checkSameAsLast(first));
    tracker.updateLast(first);
    assert(tracker.checkSameAsLast(first));
    assert(!tracker.checkSameAsLast(changed));
    tracker.updateLast(changed);
    assert(tracker.checkSameAsLast(changed));
    assert(!tracker.checkSameAsLast(first));
    tracker.clearLasts();
    assert(!tracker.checkSameAsLast(changed));
}

void testStreamCaches() {
    using namespace LogProtocol;
    testCache(IMUEvent{}, IMUEvent{1, 0, 0, 0, 0, 0});
    testCache(BarometerEvent{101325, 20}, BarometerEvent{101325, 21});
    testCache(BarometricAltitudeEvent{0}, BarometricAltitudeEvent{1});
    testCache(GimbalPositionEvent{}, GimbalPositionEvent{0, 0, 0, 1});
    testCache(RocketRotationEvent{Eigen::Quaternionf::Identity()}, RocketRotationEvent{Eigen::Quaternionf(0, 1, 0, 0)});
    testCache(RocketPositionEvent{}, RocketPositionEvent{0, 1, 0});
    testCache(BatteryVoltageEvent{3}, BatteryVoltageEvent{4});
    testCache(PyroArmStateEvent{}, PyroArmStateEvent{false, true});
    testCache(PyroContinuityEvent{0, false}, PyroContinuityEvent{0, true});
    testCache(PIDEvent{}, PIDEvent{true, 0, 0, 0, 0, 0, 0, false, false});
    testCache(VelocityEvent{}, VelocityEvent{0, 0, 0, true, false});
    testCache(SensorHealthEvent{SensorId::IMU, true, false, SensorError::None},
              SensorHealthEvent{SensorId::IMU, true, true, SensorError::Read});
    testCache(RadioDiagnosticsEvent{}, RadioDiagnosticsEvent{0, 0, 0, 1});

    LastEventsTracker tracker;
    tracker.updateLast(BatteryVoltageEvent{3.751f});
    assert(tracker.checkSameAsLast(BatteryVoltageEvent{3.759f})); // same encoded 3.75
    for (uint8_t channel = 0; channel < 3; ++channel) tracker.updateLast(PyroContinuityEvent{channel, true});
    for (uint8_t channel = 0; channel < 3; ++channel) assert(tracker.checkSameAsLast(PyroContinuityEvent{channel, true}));
    tracker.updateLast(SensorHealthEvent{SensorId::IMU, true, false, SensorError::None});
    tracker.updateLast(SensorHealthEvent{SensorId::Barometer, true, false, SensorError::None});
    assert(tracker.checkSameAsLast(SensorHealthEvent{SensorId::IMU, true, false, SensorError::None}));
    tracker.clearLasts();
    for (uint8_t channel = 0; channel < 3; ++channel) assert(!tracker.checkSameAsLast(PyroContinuityEvent{channel, true}));
}

class TestIMU : public hardware::IMU {
public:
    Accel accel{1.25f, -2.5f, 9.75f};
    Gyro gyro{0.25f, -0.5f, 0.75f};
    mutable size_t reads = 0;
    void begin() override {}
    void update() override {}
    Accel getAccel() const override { ++reads; return accel; }
    Gyro getGyro() const override { return gyro; }
};
class TestBarometer : public hardware::Barometer {
public:
    float pressure = 101325;
    float temperature = 20;
    void begin() override {}
    void update() override {}
    float getPressure_Pa() const override { return pressure; }
    float getTemperature_C() const override { return temperature; }
};

struct TelemetryFixture {
    MemoryFlash flash;
    StorageManager storage{flash};
    LogManager logger{storage};
    hardware::Battery battery{10};
    TestIMU imu;
    TestBarometer barometer;
    IMURocketCoordinateConverter converter{Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity()};
    RotationAccumulator rotation{imu, converter, 10000};
    BarometricHeightCalculator height{barometer};
    VerticalMovementTracker vertical{height};
    HorizontalMovementTracker horizontal{vertical, rotation};
    hardware::Servo pitch{11, 1, -90, 90, 0, 500, 2500, 90};
    hardware::Servo yaw{12, 2, -90, 90, 0, 500, 2500, 90};
    hardware::Gimbal gimbal{pitch, yaw, {0, 0}, -5, 5, -5, 5, 1, 1, 0, 0};
    ControlPID pid{rotation, gimbal};
    TelemetryRecorder recorder{logger, battery, imu, barometer, pid, height, vertical, horizontal, gimbal, rotation};
    TelemetryFixture() {
        assert(storage.begin());
        assert(height.calibrateTo(0));
        hostArduino::pins[10] = 950;
    }
};

void testTelemetryRecorder() {
    TelemetryFixture f;
    hostArduino::nowMillis = 1000;
    hostArduino::nowMicros = 1000000;
    f.recorder.update();
    assert(f.imu.reads == 0); // don't poll sensors when not logging
    assert(f.logger.startLog(testMetadata, StorageManager::Filename("telemetry.bin")));
    f.recorder.update();
    assert(f.imu.reads == 1);
    const size_t firstSize = f.storage.bufferedBytes();
    assert(firstSize == LogProtocol::headerSize + 152); // all nine streams
    f.imu.accel.x_m_s2 = 2.5f;
    hostArduino::nowMillis += 19;
    hostArduino::nowMicros += 19000;
    f.recorder.update();
    assert(f.imu.reads == 1 && f.storage.bufferedBytes() == firstSize);
    ++hostArduino::nowMillis;
    hostArduino::nowMicros += 1000;
    f.recorder.update();
    assert(f.imu.reads == 2 && f.storage.bufferedBytes() == firstSize + 27);
    hostArduino::nowMillis += 20;
    hostArduino::nowMicros += 20000;
    f.recorder.update();
    assert(f.imu.reads == 3 && f.storage.bufferedBytes() == firstSize + 27); // unchanged = no writes
    assert(f.logger.finishLog());
    std::array<uint8_t, 152> records{};
    assert(f.storage.readFile(StorageManager::Filename("telemetry.bin"), LogProtocol::headerSize, records) == records.size());
    assert(records[0] == static_cast<uint8_t>(LogProtocol::EventType::IMU));
    assert(fixedPoint::decode32(records, 3) == 1.25f);
    assert(records[27] == static_cast<uint8_t>(LogProtocol::EventType::Barometer));
    assert(fixedPoint::decode32(records, 30) == 101325);
    // Restart without an idle update: session ID must bypass the 20ms gate.
    assert(f.logger.startLog(testMetadata, StorageManager::Filename("restart.bin")));
    f.recorder.update();
    assert(f.imu.reads == 4 && f.storage.bufferedBytes() == firstSize);
    assert(f.logger.finishLog());
    f.recorder.update();
    assert(f.imu.reads == 4);
    // unsigned elapsed-time arithmetic must also work across millis rollover.
    hostArduino::nowMillis = UINT32_MAX - 10;
    assert(f.logger.startLog(testMetadata, StorageManager::Filename("rollover.bin")));
    f.recorder.update();
    hostArduino::nowMillis = 8;
    f.recorder.update();
    assert(f.imu.reads == 5);
    hostArduino::nowMillis = 9;
    f.recorder.update();
    assert(f.imu.reads == 6);
    assert(f.logger.finishLog());
}

void testFailedWriteRetries() {
    TelemetryFixture f;
    assert(f.logger.startLog(testMetadata, StorageManager::Filename("retry.bin")));
    // Leave too little buffer space for a sample; force the necessary flush to fail.
    std::array<uint8_t, 764> filler{};
    assert(f.storage.write(filler) == StorageManager::WriteResult::Ok);
    f.flash.failWrites = true;
    assert(!f.logger.appendEvent(LogProtocol::BatteryVoltageEvent{3.75f}));
    f.flash.failWrites = false;
    assert(f.logger.appendEvent(LogProtocol::BatteryVoltageEvent{3.75f}));
    const size_t bytes = f.storage.bufferedBytes();
    assert(f.logger.appendEvent(LogProtocol::BatteryVoltageEvent{3.75f}));
    assert(f.storage.bufferedBytes() == bytes); // only successful write cached
    assert(f.logger.finishLog());
}

void testSensorHealth() {
    TelemetryFixture f;
    hostArduino::nowMillis = 2000;
    assert(f.logger.startLog(testMetadata, StorageManager::Filename("health.bin")));
    SPIClass spi;
    hardware::MS5611 barometer(20, spi, f.logger);
    barometer.begin();
    const size_t initial = f.storage.bufferedBytes();
    barometer.update();
    assert(f.storage.bufferedBytes() == initial + 7); // health only, no raw telemetry
    barometer.update();
    assert(f.storage.bufferedBytes() == initial + 7);
    MS5611_SPI::result = -10;
    barometer.update();
    assert(f.storage.bufferedBytes() == initial + 14); // read fault, not stale yet
    hostArduino::nowMillis += 100;
    barometer.update();
    const size_t staleSize = f.storage.bufferedBytes();
    barometer.update();
    assert(f.storage.bufferedBytes() == staleSize);
    MS5611_SPI::result = 0;
    MS5611_SPI::pressure = std::numeric_limits<float>::quiet_NaN();
    barometer.update();
    assert(barometer.getPressure_Pa() == 101325); // retain last good reading
    MS5611_SPI::pressure = 101325;
    barometer.update();
    const size_t recoveredSize = f.storage.bufferedBytes();
    barometer.update();
    assert(f.storage.bufferedBytes() == recoveredSize);

    hardware::ICM45686 imu(21, spi, f.logger);
    // Identity check then a valid zero burst. No begin needed for this driver test.
    spi.replies = {0, 0xE9, 0};
    spi.replies.resize(15, 0);
    imu.update();
    const size_t imuSize = f.storage.bufferedBytes();
    assert(imuSize == recoveredSize + 7); // no raw IMU record from driver
    spi.replies.clear(); // default 0xFF: invalid burst, then stale
    spi.cursor = 0;
    imu.update();
    hostArduino::nowMillis += 100;
    imu.update();
    assert(!imu.isConnected());
    const size_t disconnectedSize = f.storage.bufferedBytes();
    imu.update();
    assert(f.storage.bufferedBytes() == disconnectedSize);
    assert(f.logger.finishLog());
}

void testPIDAndFlightDetection() {
    TelemetryFixture f;
    hostArduino::nowMillis = 4000;
    hostArduino::nowMicros = 4000000;
    f.pid.setPIDParameters(10, 0, 0);
    f.pid.setTarget(Eigen::Quaternionf(Eigen::AngleAxisf(0.5f, Eigen::Vector3f::UnitX())));
    f.pid.startControlling();
    f.pid.update();
    hostArduino::nowMicros += 10000;
    f.pid.update();
    const auto pid = f.pid.getDiagnostics();
    assert(pid.controlling && pid.pitchClamped && !pid.yawClamped);
    assert(std::abs(pid.yawError_rad - 0.5f) < 0.001f && pid.appliedPitch_deg == 5);
    f.pid.stopControlling();
    assert(!f.pid.getDiagnostics().controlling);

    assert(f.logger.startLog(testMetadata, StorageManager::Filename("flight.bin")));
    PyroArmManager arm(1, &f.logger);
    hostArduino::pins[1] = HIGH;
    arm.softwareArm();
    PyroChannel motorPyro(2, 3, arm, &f.logger, 0);
    PyroChannel chutePyro(4, 5, arm, &f.logger, 1);
    hostArduino::pins[3] = hostArduino::pins[5] = HIGH;
    MotorIgniter motor;
    Parachute chute;
    FlightStateManager flight(f.pid, f.rotation, f.height, f.vertical, f.horizontal, motor, chute, f.logger);
    const FlightProfile profile{0, 0, Eigen::Quaternionf::Identity(), Eigen::Quaternionf::Identity(),
                                1, 0, 0, &motorPyro, &chutePyro, 0};
    assert(flight.startCountdown(profile));
    flight.update();
    assert(flight.getCurrentState() == FlightState::BURNING);
    flight.update();
    assert(flight.getCurrentState() == FlightState::COASTING);
    f.vertical.update();
    // Falling height over >= 100ms produces an apogee/landing velocity estimate.
    hostArduino::nowMillis += 100;
    hostArduino::nowMicros += 100000;
    f.barometer.pressure += 5;
    f.vertical.update();
    assert(f.vertical.hasVelocityEstimate());
    const size_t beforeApogee = f.storage.bufferedBytes();
    flight.update();
    assert(flight.getCurrentState() == FlightState::DESCENDING);
    assert(f.storage.bufferedBytes() >= beforeApogee + 12);
    flight.update();
    assert(flight.getCurrentState() == FlightState::LANDED);
    const size_t landedSize = f.storage.bufferedBytes();
    flight.update();
    assert(f.storage.bufferedBytes() == landedSize);
    assert(f.logger.finishLog());
}

void testRadioFaults() {
    TelemetryFixture f;
    assert(f.logger.startLog(testMetadata, StorageManager::Filename("radio.bin")));
    Protocol::Parser parser;
    TestRadio radio;
    MessageScheduler scheduler(parser, radio, {}, &f.logger);
    radio.enqueue(Protocol::MessageType::PING, 1);
    radio.incoming.back() ^= 0x80; // bad CRC
    const std::array<uint8_t, 4> malformed{Protocol::START_BYTE, 1, 0, 0};
    radio.incoming.insert(radio.incoming.end(), malformed.begin(), malformed.end());
    scheduler.update();
    assert(parser.crcErrorCount() == 1 && parser.overflowCount() == 1);
    const size_t bytes = f.storage.bufferedBytes();
    scheduler.update();
    assert(f.storage.bufferedBytes() == bytes); // unchanged counters suppressed
    assert(f.logger.finishLog());
}

void testPyroStateAndTimeout() {
    TelemetryFixture f;
    hostArduino::nowMillis = 0; // firing must work even at the zero clock value
    assert(f.logger.startLog(testMetadata, StorageManager::Filename("pyro.bin")));
    PyroArmManager arm(1, &f.logger);
    PyroChannel pyro(2, 3, arm, &f.logger, 0);
    hostArduino::pins[1] = HIGH;
    hostArduino::pins[3] = HIGH;
    arm.update();
    const size_t armBytes = f.storage.bufferedBytes();
    arm.update();
    assert(f.storage.bufferedBytes() == armBytes);
    arm.softwareArm();
    assert(f.storage.bufferedBytes() == armBytes + 5);
    assert(pyro.fire(10));
    pyro.update();
    assert(hostArduino::pins[2] == HIGH);
    hostArduino::nowMillis = 10;
    pyro.update();
    assert(hostArduino::pins[2] == LOW);
    const size_t stoppedBytes = f.storage.bufferedBytes();
    pyro.stopFiring();
    pyro.update();
    assert(f.storage.bufferedBytes() == stoppedBytes); // no phantom repeated stops
    hostArduino::pins[3] = LOW;
    assert(!pyro.fire(10));
    assert(!pyro.fire(10)); // repeated refused actions must not be suppressed
    assert(f.storage.bufferedBytes() == stoppedBytes + 18);
    assert(f.logger.finishLog());
    std::array<uint8_t, 32> records{};
    assert(f.storage.readFile(StorageManager::Filename("pyro.bin"), LogProtocol::headerSize, records) == records.size());
    assert(records[23] == static_cast<uint8_t>(LogProtocol::EventType::PyroStopped));
    assert(records[31] == static_cast<uint8_t>(LogProtocol::PyroStopReason::DurationElapsed));
}

int main() {
    testEncoding();
    testSourceEvents();
    testStreamCaches();
    testTelemetryRecorder();
    testFailedWriteRetries();
    testSensorHealth();
    testPIDAndFlightDetection();
    testRadioFaults();
    testPyroStateAndTimeout();
    std::cout << "Flight log event tests passed\n";
}