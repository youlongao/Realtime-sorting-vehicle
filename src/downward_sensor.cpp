#include "downward_sensor.h"

#include <chrono>
#include <cstring>
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

DownwardSensor::DownwardSensor(std::string chip_path,
							   const unsigned int line_offset,
							   const bool active_on_surface)
	: chip_path_(std::move(chip_path)),
	  line_offset_(line_offset),
	  active_on_surface_(active_on_surface)
{
}

DownwardSensor::~DownwardSensor()
{
	stop();
}

bool DownwardSensor::start()
{
	if (running_.load())
	{
		return true;
	}

	if (!initialiseRequest())
	{
		return false;
	}

	running_.store(true);
	worker_ = std::thread(&DownwardSensor::workerLoop, this);
	Logger::info("Downward sensor edge thread started.");
	return true;
}

void DownwardSensor::stop()
{
	if (!running_.exchange(false))
	{
		releaseRequest();
		return;
	}

	edge_cv_.notify_all();

	if (worker_.joinable())
	{
		worker_.join();
	}

	releaseRequest();
}

DownwardReading DownwardSensor::latest() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return latest_reading_;
}

bool DownwardSensor::waitForEdge(const std::chrono::milliseconds timeout)
{
	std::unique_lock<std::mutex> lock(mutex_);
	const bool edge_seen = edge_cv_.wait_for(lock, timeout, [this]() {
		return pending_edge_ || !running_.load();
	});

	if (!edge_seen || !pending_edge_)
	{
		return false;
	}

	pending_edge_ = false;
	return true;
}

void DownwardSensor::setCallback(DownwardCallback callback)
{
	std::lock_guard<std::mutex> lock(mutex_);
	callback_ = std::move(callback);
}

void DownwardSensor::updateFromValue(const bool sensor_active)
{
	DownwardCallback callback;
	DownwardReading snapshot;

	{
		std::lock_guard<std::mutex> lock(mutex_);
		latest_reading_.on_step_surface = (sensor_active == active_on_surface_);
		latest_reading_.drop_detected = !latest_reading_.on_step_surface;
		latest_reading_.edge_detected = latest_reading_.drop_detected;
		latest_reading_.timestamp = SteadyClock::now();
		latest_reading_.valid = true;
		pending_edge_ = true;
		callback = callback_;
		snapshot = latest_reading_;
	}

	edge_cv_.notify_all();

	if (callback)
	{
		callback(snapshot);
	}
}

void DownwardSensor::workerLoop()
{
	while (running_.load())
	{
		if (request_ == nullptr)
		{
			break;
		}

		const int wait_result = gpiod_line_request_wait_edge_events(
			request_,
			std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::milliseconds(100)).count());

		if (!running_.load())
		{
			break;
		}

		if (wait_result < 0)
		{
			Logger::error(FormatErrno("Downward sensor wait failed"));
			break;
		}

		if (wait_result == 0)
		{
			continue;
		}

		const int events_read = gpiod_line_request_read_edge_events(
			request_,
			event_buffer_,
			RobotConfig::Sensors::ECHO_EVENT_BUFFER_SIZE);

		if (events_read < 0)
		{
			Logger::error(FormatErrno("Downward sensor edge read failed"));
			continue;
		}

		for (int index = 0; index < events_read; ++index)
		{
			const auto* event = gpiod_edge_event_buffer_get_event(event_buffer_, index);
			if (event == nullptr)
			{
				continue;
			}

			const auto value = gpiod_line_request_get_value(request_, line_offset_);
			updateFromValue(value == GPIOD_LINE_VALUE_ACTIVE);
		}
	}
}

bool DownwardSensor::initialiseRequest()
{
	chip_ = gpiod_chip_open(chip_path_.c_str());
	if (chip_ == nullptr)
	{
		Logger::error(FormatErrno("Failed to open gpiochip for downward sensor"));
		return false;
	}

	auto* settings = gpiod_line_settings_new();
	auto* line_config = gpiod_line_config_new();
	auto* request_config = gpiod_request_config_new();

	if (settings == nullptr || line_config == nullptr || request_config == nullptr)
	{
		Logger::error("Failed to allocate libgpiod objects for downward sensor.");
		gpiod_line_settings_free(settings);
		gpiod_line_config_free(line_config);
		gpiod_request_config_free(request_config);
		releaseRequest();
		return false;
	}

	gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
	gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_BOTH);
	gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_DISABLED);
	gpiod_line_settings_set_active_low(settings, false);
	gpiod_line_settings_set_debounce_period_us(settings, RobotConfig::Sensors::DOWNWARD_DEBOUNCE_US);
	gpiod_line_settings_set_event_clock(settings, GPIOD_LINE_CLOCK_MONOTONIC);

	const unsigned int offset = line_offset_;
	gpiod_line_config_add_line_settings(line_config, &offset, 1, settings);

	gpiod_request_config_set_consumer(request_config, "downward-sensor");
	gpiod_request_config_set_event_buffer_size(
		request_config,
		RobotConfig::Sensors::ECHO_EVENT_BUFFER_SIZE);

	request_ = gpiod_chip_request_lines(chip_, request_config, line_config);
	event_buffer_ = gpiod_edge_event_buffer_new(RobotConfig::Sensors::ECHO_EVENT_BUFFER_SIZE);

	gpiod_line_settings_free(settings);
	gpiod_line_config_free(line_config);
	gpiod_request_config_free(request_config);

	if (request_ == nullptr || event_buffer_ == nullptr)
	{
		Logger::error(FormatErrno("Failed to request downward sensor GPIO line"));
		releaseRequest();
		return false;
	}

	const auto current_value = gpiod_line_request_get_value(request_, line_offset_);
	updateFromValue(current_value == GPIOD_LINE_VALUE_ACTIVE);
	return true;
}

void DownwardSensor::releaseRequest()
{
	if (event_buffer_ != nullptr)
	{
		gpiod_edge_event_buffer_free(event_buffer_);
		event_buffer_ = nullptr;
	}

	if (request_ != nullptr)
	{
		gpiod_line_request_release(request_);
		request_ = nullptr;
	}

	if (chip_ != nullptr)
	{
		gpiod_chip_close(chip_);
		chip_ = nullptr;
	}
}
}
