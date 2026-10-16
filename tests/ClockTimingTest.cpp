#include <Arduino.h>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include "timing/Clock.hpp"
#include "simMode/timing/SimClock.hpp"
#include "rotationEstimation/RotationAccumulator.hpp"
#include "hardwareIO/pyro/PyroChannel.hpp"
#include "hardwareIO/pyro/PyroArmManager.hpp"
#include "logManagement/LogManager.hpp"
#include "radioLink/requestHandlers/logs/DownloadManager.hpp"

class ConstantIMU : public hardware::IMU {
public:
    void begin() override {}
    void update() override {}
    Accel getAccel() const override { return {0, 0, 0}; }
    Gyro getGyro() const override { return {0, 0, 1}; }
};

class MemoryFlash : public hardware::FlashChip {
public:
    std::vector<uint8_t> bytes = std::vector<uint8_t>(1024 * 1024, 0xff);
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
        if (address > bytes.size() || data.size() > bytes.size() - address) return false;
        for (size_t i = 0; i < data.size(); ++i) bytes[address + i] &= data[i];
        return true;
    }
    bool eraseSector(uint32_t address) override {
        if (address % 4096 || address > bytes.size() - 4096) return false;
        std::fill_n(bytes.begin() + address, 4096, 0xff);
        return true;
    }
    bool eraseChip() override {
        std::fill(bytes.begin(), bytes.end(), 0xff);
        return true;
    }
};

static void testRotationFlightTime() {
    SimClock clock;
    ConstantIMU imu;
    IMURocketCoordinateConverter converter(Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity());
    RotationAccumulator rotation(clock, imu, converter, 1000000);
    rotation.startAccumulation(); // Flight time zero is a valid starting timestamp.
    clock.advanceTimeBy_us(100000);
    rotation.update();
    assert(std::abs(rotation.getEulerAngles_rad().z() - 0.1f) < 1e-5f);
    hostArduino::nowMillis += 1000;
    rotation.update(); // Real time alone must not rotate a paused simulation.
    assert(std::abs(rotation.getEulerAngles_rad().z() - 0.1f) < 1e-5f);
    clock.advanceTimeBy_us(200000);
    rotation.update();
    assert(std::abs(rotation.getEulerAngles_rad().z() - 0.3f) < 1e-5f);

    clock.advanceTimeBy_us(std::numeric_limits<uint32_t>::max() - clock.getTimeFlight_us() - 50000);
    rotation.startAccumulation();
    clock.advanceTimeBy_us(100000); // Unsigned flight-time rollover.
    rotation.update();
    assert(std::abs(rotation.getEulerAngles_rad().z() - 0.4f) < 1e-5f);
}

static void testPhysicalPyroRealTime() {
    SimClock clock;
    hardware::PyroArmManager arm(1);
    hostArduino::pins[1] = hostArduino::pins[3] = HIGH;
    arm.softwareArm();
    hardware::PyroChannel pyro(clock, 2, 3, arm);
    hostArduino::nowMillis = std::numeric_limits<uint32_t>::max() - 50;
    assert(pyro.fire(100));
    clock.advanceTimeBy_ms(100000);
    pyro.update();
    assert(hostArduino::pins[2] == HIGH);
    hostArduino::nowMillis += 99;
    pyro.update();
    assert(hostArduino::pins[2] == HIGH);
    hostArduino::nowMillis += 1;
    pyro.update();
    assert(hostArduino::pins[2] == LOW);
}

static void testLogFlightTimeAndDownloadRealTime() {
    SimClock clock;
    MemoryFlash flash;
    logging::StorageManager storage(flash);
    assert(storage.begin());
    logging::LogManager logger(clock, storage);
    const logging::StorageManager::Filename filename("clock.bin");
    assert(logger.startLog({0, "clock"}, filename));
    assert(logger.appendEvent(LogProtocol::BatteryEvent{12.0f}));
    clock.advanceTimeBy_us(12345);
    hostArduino::nowMillis += 5000;
    assert(logger.appendEvent(LogProtocol::BatteryEvent{11.0f}));
    assert(logger.finishLog());
    std::array<uint8_t, 14> events{};
    assert(storage.readFile(filename, LogProtocol::headerSize, events) == events.size());
    assert(littleEndian::decodeU16(events, 8) == 12345);

    radioLink::DownloadManager download(clock, storage);
    std::array<uint8_t, 40> request{};
    const auto encoded = stringCodec::encode(filename, request, 0);
    assert(encoded);
    littleEndian::encodeU32(1, request, *encoded);
    const auto start = std::span(request).first(*encoded + 4);
    std::array<uint8_t, 256> response{};
    const auto success = radioLink::MessageScheduler::HandlerResultStatus::SUCCESS;
    const auto failure = radioLink::MessageScheduler::HandlerResultStatus::FAILURE;
    assert(download.start(start, response).status == success);
    const auto session = littleEndian::decodeU32(response, 0);
    clock.advanceTimeBy_ms(120000);
    download.update();
    assert(download.start(start, response).status == success);
    assert(littleEndian::decodeU32(response, 0) == session);
    hostArduino::nowMillis += radioLink::DownloadManager::TIMEOUT_MS;
    download.update();
    std::array<uint8_t, 8> chunk{};
    littleEndian::encodeU32(session, chunk, 0);
    assert(download.getChunk(chunk, response).status == failure);
}

int main() {
    Clock realClock;
    assert(realClock.getTimeFlight_ms() == realClock.getTime_ms());
    assert(realClock.getTimeFlight_us() == realClock.getTime_us());
    testRotationFlightTime();
    testPhysicalPyroRealTime();
    testLogFlightTimeAndDownloadRealTime();
    std::cout << "Clock timing tests passed\n";
}