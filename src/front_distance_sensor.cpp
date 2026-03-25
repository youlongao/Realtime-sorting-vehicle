#include "front_distance_sensor.h"

#include <chrono>
#include <cstring>
#include <thread>
#include <utility>

#include "logger.h"

#include <cerrno>

#include <gpiod.h>

namespace Robot
{
namespace
{
std::string FormatErrno(const std::string& prefix)
{
	return prefix + ": " + std::strerror(errno);
}
}

FrontDistanceSensor::FrontDistanceSensor(std::string chip_path,
										 const unsigned int trig_offset,
										 const unsigned int echo_offset)
	: chip_path_(std::move(chip_path)),
	  trig_offset_(trig_offset),
	  echo_offset_(echo_offset)
{
}

FrontDistanceSensor::~FrontDistanceSensor()
{
	stop();
}

bool FrontDistanceSensor::start()
{
	if (trig_request_ != nullptr && echo_request_ != nullptr)
	{
		return true;
	}

	return initialiseRequests();
}

void FrontDistanceSensor::stop()
{
	releaseRequests();
}

DistanceReading FrontDistanceSensor::readBlocking(const std::chrono::microseconds timeout)
{
	if ((trig_request_ == nullptr || echo_request_ == nullptr) && !initialiseRequests())
	{
		return {};
	}

	clearPendingEchoEvents();

	if (gpiod_line_request_set_value(trig_request_, trig_offset_, GPIOD_LINE_VALUE_ACTIVE) < 0)
	{
		Logger::error(FormatErrno("Failed to set ultrasonic trigger high"));
		return {};
	}

	std::this_thread::sleep_for(std::chrono::microseconds(10));

	if (gpiod_line_request_set_value(trig_request_, trig_offset_, GPIOD_LINE_VALUE_INACTIVE) < 0)
	{
		Logger::error(FormatErrno("Failed to set ultrasonic trigger low"));
		return {};
	}

	const auto deadline = SteadyClock::now() + timeout;
	bool saw_rising_edge = false;
	std::uint64_t rising_ns = 0;
	std::uint64_t falling_ns = 0;

	while (SteadyClock::now() < deadline)
	{
		const auto remaining =
			std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - SteadyClock::now()).count();
		const int wait_result = gpiod_line_request_wait_edge_events(echo_request_, remaining);

		if (wait_result < 0)
		{
			Logger::error(FormatErrno("Ultrasonic echo wait failed"));
			break;
		}

		if (wait_result == 0)
		{
			break;
		}

		const int events_read = gpiod_line_request_read_edge_events(
			echo_request_,
			event_buffer_,
			RobotConfig::Sensors::ECHO_EVENT_BUFFER_SIZE);

		if (events_read < 0)
		{
			Logger::error(FormatErrno("Ultrasonic echo edge read failed"));
			break;
		}

		for (int index = 0; index < events_read; ++index)
		{
			const auto* event = gpiod_edge_event_buffer_get_event(event_buffer_, index);
			if (event == nullptr)
			{
				continue;
			}

			const auto type = gpiod_edge_event_get_event_type(event);
			const auto timestamp_ns = gpiod_edge_event_get_timestamp_ns(event);

			if (type == GPIOD_EDGE_EVENT_RISING_EDGE)
			{
				saw_rising_edge = true;
				rising_ns = timestamp_ns;
			}
			else if (type == GPIOD_EDGE_EVENT_FALLING_EDGE && saw_rising_edge)
			{
				falling_ns = timestamp_ns;
				break;
			}
		}

		if (saw_rising_edge && falling_ns > rising_ns)
		{
			break;
		}
	}

	DistanceReading result;
	result.timestamp = SteadyClock::now();
	result.valid = saw_rising_edge && falling_ns > rising_ns;

	if (result.valid)
	{
		result.distance_m = pulseWidthToDistance(falling_ns - rising_ns);
	}

	DistanceCallback callback;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		latest_reading_ = result;
		callback = callback_;
	}

	if (callback)
	{
		callback(result);
	}

	return result;
}

DistanceReading FrontDistanceSensor::latest() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return latest_reading_;
}

void FrontDistanceSensor::setCallback(DistanceCallback callback)
{
	std::lock_guard<std::mutex> lock(mutex_);
	callback_ = std::move(callback);
}

float FrontDistanceSensor::pulseWidthToDistance(const std::uint64_t pulse_width_ns) const
{
	const float pulse_width_seconds = static_cast<float>(pulse_width_ns) / 1'000'000'000.0F;
	return (pulse_width_seconds * RobotConfig::Sensors::SPEED_OF_SOUND_MPS) * 0.5F;
}

bool FrontDistanceSensor::initialiseRequests()
{
	if (trig_request_ != nullptr && echo_request_ != nullptr)
	{
		return true;
	}

	chip_ = gpiod_chip_open(chip_path_.c_str());
	if (chip_ == nullptr)
	{
		Logger::error(FormatErrno("Failed to open gpiochip for ultrasonic sensor"));
		return false;
	}

	auto* trig_settings = gpiod_line_settings_new();
	auto* trig_config = gpiod_line_config_new();
	auto* trig_request_config = gpiod_request_config_new();
	auto* echo_settings = gpiod_line_settings_new();
	auto* echo_config = gpiod_line_config_new();
	auto* echo_request_config = gpiod_request_config_new();

	if (trig_settings == nullptr || trig_config == nullptr || trig_request_config == nullptr ||
		echo_settings == nullptr || echo_config == nullptr || echo_request_config == nullptr)
	{
		Logger::error("Failed to allocate libgpiod objects for ultrasonic sensor.");
		gpiod_line_settings_free(trig_settings);
		gpiod_line_config_free(trig_config);
		gpiod_request_config_free(trig_request_config);
		gpiod_line_settings_free(echo_settings);
		gpiod_line_config_free(echo_config);
		gpiod_request_config_free(echo_request_config);
		releaseRequests();
		return false;
	}

	gpiod_line_settings_set_direction(trig_settings, GPIOD_LINE_DIRECTION_OUTPUT);
	gpiod_line_settings_set_output_value(trig_settings, GPIOD_LINE_VALUE_INACTIVE);
	gpiod_line_settings_set_drive(trig_settings, GPIOD_LINE_DRIVE_PUSH_PULL);
	gpiod_request_config_set_consumer(trig_request_config, "ultrasonic-trig");

	gpiod_line_settings_set_direction(echo_settings, GPIOD_LINE_DIRECTION_INPUT);
	gpiod_line_settings_set_edge_detection(echo_settings, GPIOD_LINE_EDGE_BOTH);
	gpiod_line_settings_set_bias(echo_settings, GPIOD_LINE_BIAS_DISABLED);
	gpiod_line_settings_set_event_clock(echo_settings, GPIOD_LINE_CLOCK_MONOTONIC);
	gpiod_request_config_set_consumer(echo_request_config, "ultrasonic-echo");
	gpiod_request_config_set_event_buffer_size(
		echo_request_config,
		RobotConfig::Sensors::ECHO_EVENT_BUFFER_SIZE);

	const unsigned int trig_offset = trig_offset_;
	const unsigned int echo_offset = echo_offset_;
	gpiod_line_config_add_line_settings(trig_config, &trig_offset, 1, trig_settings);
	gpiod_line_config_add_line_settings(echo_config, &echo_offset, 1, echo_settings);

	trig_request_ = gpiod_chip_request_lines(chip_, trig_request_config, trig_config);
	echo_request_ = gpiod_chip_request_lines(chip_, echo_request_config, echo_config);
	event_buffer_ = gpiod_edge_event_buffer_new(RobotConfig::Sensors::ECHO_EVENT_BUFFER_SIZE);

	gpiod_line_settings_free(trig_settings);
	gpiod_line_config_free(trig_config);
	gpiod_request_config_free(trig_request_config);
	gpiod_line_settings_free(echo_settings);
	gpiod_line_config_free(echo_config);
	gpiod_request_config_free(echo_request_config);

	if (trig_request_ == nullptr || echo_request_ == nullptr || event_buffer_ == nullptr)
	{
		Logger::error(FormatErrno("Failed to request ultrasonic GPIO lines"));
		releaseRequests();
		return false;
	}

	return true;
}

void FrontDistanceSensor::releaseRequests()
{
	if (event_buffer_ != nullptr)
	{
		gpiod_edge_event_buffer_free(event_buffer_);
		event_buffer_ = nullptr;
	}

	if (echo_request_ != nullptr)
	{
		gpiod_line_request_release(echo_request_);
		echo_request_ = nullptr;
	}

	if (trig_request_ != nullptr)
	{
		gpiod_line_request_release(trig_request_);
		trig_request_ = nullptr;
	}

	if (chip_ != nullptr)
	{
		gpiod_chip_close(chip_);
		chip_ = nullptr;
	}
}

void FrontDistanceSensor::clearPendingEchoEvents()
{
	while (gpiod_line_request_wait_edge_events(echo_request_, 0) > 0)
	{
		gpiod_line_request_read_edge_events(
			echo_request_,
			event_buffer_,
			RobotConfig::Sensors::ECHO_EVENT_BUFFER_SIZE);
	}
}
}
