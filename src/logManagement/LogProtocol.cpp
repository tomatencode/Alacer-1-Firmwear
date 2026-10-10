#include "LogProtocol.hpp"

#include <algorithm>
#include <cassert>

#include "../helpers/codec/FixedPointCodec.hpp"
#include "../helpers/codec/LittleEndianCodec.hpp"
#include "../helpers/codec/QuaternionCodec.hpp"

namespace {

constexpr size_t kTimeSyncSize = 5;
constexpr size_t kDroppedEventsSize = 9;
constexpr size_t kUpdateCycleDoneEventSize = 3;
constexpr size_t kImuEventSize = 27;
constexpr size_t kBarometerEventSize = 11;

void requireBufferSize(std::span<uint8_t> buffer, size_t requiredSize) {
	assert(buffer.size() >= requiredSize);
}

void encodeEventPrefix(LogProtocol::EventType type, uint16_t timestampDeltaUs,
					   std::span<uint8_t> buffer) {
	buffer[0] = static_cast<uint8_t>(type);
	littleEndian::encodeU16(timestampDeltaUs, buffer, 1);
}

} // namespace

size_t LogProtocol::encodeHeader(const LogMetadata& metadata, std::span<uint8_t> buffer) {
	requireBufferSize(buffer, headerSize);
	std::fill_n(buffer.begin(), headerSize, 0);

	littleEndian::encodeU32(metadata.timestamp_unix, buffer, 0);
	quaternionCodec::encode(metadata.initialRotation, buffer, 4);
	quaternionCodec::encode(metadata.targetAngle, buffer, 20);
	fixedPoint::encode32(metadata.pidKp, buffer, 36);
	fixedPoint::encode32(metadata.pidKi, buffer, 40);
	fixedPoint::encode32(metadata.pidKd, buffer, 44);
	fixedPoint::encode32(metadata.initialHeight_m, buffer, 48);
	return headerSize;
}

size_t LogProtocol::encodeTimeSync(uint32_t time_since_start_ms, std::span<uint8_t> buffer) {
	requireBufferSize(buffer, kTimeSyncSize);
	buffer[0] = static_cast<uint8_t>(EventType::TimeSync);
	littleEndian::encodeU32(time_since_start_ms, buffer, 1);
	return kTimeSyncSize;
}

size_t LogProtocol::encodeDroppedEvents(uint32_t count, uint32_t time_since_start_ms,
										std::span<uint8_t> buffer) {
	requireBufferSize(buffer, kDroppedEventsSize);
	buffer[0] = static_cast<uint8_t>(EventType::DroppedEvents);
	littleEndian::encodeU32(time_since_start_ms, buffer, 1);
	littleEndian::encodeU32(count, buffer, 5);
	return kDroppedEventsSize;
}

size_t LogProtocol::encodeEvent(const UpdateCycleDoneEvent& event, uint16_t timestamp_d_us, std::span<uint8_t> buffer) {
	requireBufferSize(buffer, kUpdateCycleDoneEventSize);
	encodeEventPrefix(EventType::UpdateCycleDone, timestamp_d_us, buffer);
	return kUpdateCycleDoneEventSize;
}

size_t LogProtocol::encodeEvent(const IMUEvent& event, uint16_t timestamp_d_us,
								std::span<uint8_t> buffer) {
	requireBufferSize(buffer, kImuEventSize);
	encodeEventPrefix(EventType::IMU, timestamp_d_us, buffer);
	fixedPoint::encode32(event.accel_x_m_s2, buffer, 3);
	fixedPoint::encode32(event.accel_y_m_s2, buffer, 7);
	fixedPoint::encode32(event.accel_z_m_s2, buffer, 11);
	fixedPoint::encode32(event.gyro_x_rad_s, buffer, 15);
	fixedPoint::encode32(event.gyro_y_rad_s, buffer, 19);
	fixedPoint::encode32(event.gyro_z_rad_s, buffer, 23);
	return kImuEventSize;
}

size_t LogProtocol::encodeEvent(const BarometerEvent& event, uint16_t timestamp_d_us,
								std::span<uint8_t> buffer) {
	requireBufferSize(buffer, kBarometerEventSize);
	encodeEventPrefix(EventType::Barometer, timestamp_d_us, buffer);
	fixedPoint::encode32(event.pressure_Pa, buffer, 3);
	fixedPoint::encode32(event.temperature_C, buffer, 7);
	return kBarometerEventSize;
}

