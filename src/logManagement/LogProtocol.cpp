#include "LogProtocol.hpp"

#include <algorithm>
#include <cassert>

#include "../helpers/codec/FixedPointCodec.hpp"
#include "../helpers/codec/LittleEndianCodec.hpp"
#include "../helpers/codec/QuaternionCodec.hpp"
#include "../helpers/codec/StringCodec.hpp"

namespace {

constexpr size_t kTimeSyncSize = 5;
constexpr size_t kDroppedEventsSize = 9;
constexpr size_t kFlightConfigurationEventSize = 51;
constexpr size_t kFlightStateChangedEventSize = 4;
constexpr size_t kUpdateCycleDoneEventSize = 3;
constexpr size_t kImuEventSize = 27;
constexpr size_t kBarometerEventSize = 11;
constexpr size_t kBatteryEventSize = 7;
constexpr size_t kGimbalEventSize = 11;
constexpr size_t kRotationEventSize = 19;
constexpr size_t kHorizontalMovementEventSize = 20;
constexpr size_t kVerticalMovementEventSize = 11;

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
	const auto nameSize = stringCodec::encode(metadata.logName, buffer, 4);
	assert(nameSize.has_value());
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


size_t LogProtocol::encodeEvent(const FlightConfigurationEvent& event, uint16_t timestamp_d_us, std::span<uint8_t> buffer) {
	requireBufferSize(buffer, kFlightConfigurationEventSize);
	encodeEventPrefix(EventType::FlightConfiguration, timestamp_d_us, buffer);
	quaternionCodec::encode(event.initialRotation, buffer, 3);
	quaternionCodec::encode(event.targetAngle, buffer, 19);
	fixedPoint::encode32(event.pidKp, buffer, 35);
	fixedPoint::encode32(event.pidKi, buffer, 39);
	fixedPoint::encode32(event.pidKd, buffer, 43);
	fixedPoint::encode32(event.initialHeight_m, buffer, 47);
	return kFlightConfigurationEventSize;
}

size_t LogProtocol::encodeEvent(const FlightStateChangedEvent& event, uint16_t timestamp_d_us, std::span<uint8_t> buffer) {
	requireBufferSize(buffer, kFlightStateChangedEventSize);
	encodeEventPrefix(EventType::FlightStateChanged, timestamp_d_us, buffer);
	buffer[3] = static_cast<uint8_t>(event.newState);
	return kFlightStateChangedEventSize;
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

size_t LogProtocol::encodeEvent(const BatteryEvent& event, uint16_t timestamp_d_us,
								std::span<uint8_t> buffer) {
	requireBufferSize(buffer, kBatteryEventSize);
	encodeEventPrefix(EventType::Battery, timestamp_d_us, buffer);
	fixedPoint::encode32(event.voltage_v, buffer, 3);
	return kBatteryEventSize;
}

size_t LogProtocol::encodeEvent(const GimbalEvent& event, uint16_t timestamp_d_us,
								std::span<uint8_t> buffer) {
	requireBufferSize(buffer, kGimbalEventSize);
	encodeEventPrefix(EventType::Gimbal, timestamp_d_us, buffer);
	fixedPoint::encode32(event.pitch_deg, buffer, 3);
	fixedPoint::encode32(event.yaw_deg, buffer, 7);
	return kGimbalEventSize;
}

size_t LogProtocol::encodeEvent(const RotationEvent& event, uint16_t timestamp_d_us,
								std::span<uint8_t> buffer) {
	requireBufferSize(buffer, kRotationEventSize);
	encodeEventPrefix(EventType::Rotation, timestamp_d_us, buffer);
	quaternionCodec::encode(event.rotation, buffer, 3);
	return kRotationEventSize;
}

size_t LogProtocol::encodeEvent(const HorizontalMovementEvent& event, uint16_t timestamp_d_us,
								std::span<uint8_t> buffer) {
	requireBufferSize(buffer, kHorizontalMovementEventSize);
	encodeEventPrefix(EventType::HorizontalMovement, timestamp_d_us, buffer);
	fixedPoint::encode32(event.x_m, buffer, 3);
	fixedPoint::encode32(event.y_m, buffer, 7);
	fixedPoint::encode32(event.velocity_x_m_s, buffer, 11);
	fixedPoint::encode32(event.velocity_y_m_s, buffer, 15);
	buffer[19] = event.ascentStage;
	return kHorizontalMovementEventSize;
}

size_t LogProtocol::encodeEvent(const VerticalMovementEvent& event, uint16_t timestamp_d_us,
								std::span<uint8_t> buffer) {
	requireBufferSize(buffer, kVerticalMovementEventSize);
	encodeEventPrefix(EventType::VerticalMovement, timestamp_d_us, buffer);
	fixedPoint::encode32(event.height_m, buffer, 3);
	fixedPoint::encode32(event.velocity_m_s, buffer, 7);
	return kVerticalMovementEventSize;
}
