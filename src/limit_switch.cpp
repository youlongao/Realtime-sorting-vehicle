#include "limit_switch.h"

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

LimitSwitch::LimitSwitch(const unsigned int line_offset,
						 const LimitRole role,
						 const bool active_low,
						 std::string chip_path)
	: line_offset_(line_offset),
	  role_(role),
	  active_low_(active_low),
	  chip_path_(std::move(chip_path))
{
}

LimitSwitch::~LimitSwitch()
{
	stop();
}

bool LimitSwitch::start()
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
	worker_ = std::thread(&LimitSwitch::workerLoop, this);
	return true;
}

void LimitSwitch::stop()
{
	if (!running_.exchange(false))
	{
		releaseRequest();
		return;
	}

	trigger_cv_.notify_all();

	if (worker_.joinable())
	{
		worker_.join();
	}

	releaseRequest();
}

bool LimitSwitch::isTriggered() const
{
	return latestState().triggered;
}

bool LimitSwitch::isUpperLimit() const
{
	return role_ == LimitRole::Upper && isTriggered();
}

bool LimitSwitch::isLowerLimit() const
{
	return role_ == LimitRole::Lower && isTriggered();
}

LimitSwitchState LimitSwitch::latestState() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return state_;
}

bool LimitSwitch::waitForTrigger(const std::chrono::milliseconds timeout)
{
	std::unique_lock<std::mutex> lock(mutex_);
	const bool notified = trigger_cv_.wait_for(lock, timeout, [this]() {
		return pending_trigger_ || !running_.load();
	});

	if (!notified || !pending_trigger_)
	{
		return false;
	}

	pending_trigger_ = false;
	return true;
}

void LimitSwitch::setCallback(LimitSwitchCallback callback)
{
	std::lock_guard<std::mutex> lock(mutex_);
	callback_ = std::move(callback);
}

void LimitSwitch::workerLoop()
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
			Logger::error(FormatErrno("Limit switch wait failed"));
			break;
		}

		if (wait_result == 0)
		{
			continue;
		}

		const int events_read = gpiod_line_request_read_edge_events(
			request_,
			event_buffer_,
			RobotConfig::Sensors::GPIO_EVENT_BUFFER_SIZE);

		if (events_read < 0)
		{
			Logger::error(FormatErrno("Limit switch edge read failed"));
			continue;
		}

		for (int index = 0; index < events_read; ++index)
		{
			(void)gpiod_edge_event_buffer_get_event(event_buffer_, index);
			const auto value = gpiod_line_request_get_value(request_, line_offset_);
			updateFromValue(value == GPIOD_LINE_VALUE_ACTIVE);
		}
	}
}

void LimitSwitch::updateFromValue(const bool line_active)
{
	LimitSwitchCallback callback;
	LimitSwitchState snapshot;

	{
		std::lock_guard<std::mutex> lock(mutex_);
		state_.triggered = active_low_ ? !line_active : line_active;
		state_.timestamp = SteadyClock::now();
		state_.valid = true;
		pending_trigger_ = state_.triggered;
		callback = callback_;
		snapshot = state_;
	}

	trigger_cv_.notify_all();

	if (callback)
	{
		callback(snapshot);
	}
}

bool LimitSwitch::initialiseRequest()
{
	chip_ = gpiod_chip_open(chip_path_.c_str());
	if (chip_ == nullptr)
	{
		Logger::error(FormatErrno("Failed to open gpiochip for limit switch"));
		return false;
	}

	auto* settings = gpiod_line_settings_new();
	auto* config = gpiod_line_config_new();
	auto* request_config = gpiod_request_config_new();

	if (settings == nullptr || config == nullptr || request_config == nullptr)
	{
		Logger::error("Failed to allocate libgpiod objects for limit switch.");
		gpiod_line_settings_free(settings);
		gpiod_line_config_free(config);
		gpiod_request_config_free(request_config);
		releaseRequest();
		return false;
	}

	gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
	gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_BOTH);
	gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_DISABLED);
	gpiod_line_settings_set_active_low(settings, false);
	gpiod_line_settings_set_event_clock(settings, GPIOD_LINE_CLOCK_MONOTONIC);

	const unsigned int offset = line_offset_;
	gpiod_line_config_add_line_settings(config, &offset, 1, settings);
	gpiod_request_config_set_consumer(request_config, "limit-switch");
	gpiod_request_config_set_event_buffer_size(request_config, RobotConfig::Sensors::GPIO_EVENT_BUFFER_SIZE);

	request_ = gpiod_chip_request_lines(chip_, request_config, config);
	event_buffer_ = gpiod_edge_event_buffer_new(RobotConfig::Sensors::GPIO_EVENT_BUFFER_SIZE);

	gpiod_line_settings_free(settings);
	gpiod_line_config_free(config);
	gpiod_request_config_free(request_config);

	if (request_ == nullptr || event_buffer_ == nullptr)
	{
		Logger::error(FormatErrno("Failed to request limit switch GPIO line"));
		releaseRequest();
		return false;
	}

	updateFromValue(gpiod_line_request_get_value(request_, line_offset_) == GPIOD_LINE_VALUE_ACTIVE);
	return true;
}

void LimitSwitch::releaseRequest()
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
