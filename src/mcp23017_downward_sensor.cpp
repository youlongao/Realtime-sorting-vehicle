#include "mcp23017_downward_sensor.h"

#include <chrono>
#include <utility>

namespace Robot
{
Mcp23017DownwardSensor::Mcp23017DownwardSensor(std::shared_ptr<Mcp23017Driver> driver,
											   const std::uint8_t pin,
											   const bool active_on_surface)
	: driver_(std::move(driver)),
	  pin_(pin),
	  active_on_surface_(active_on_surface)
{
}

Mcp23017DownwardSensor::~Mcp23017DownwardSensor()
{
	stop();
}

bool Mcp23017DownwardSensor::start()
{
	if (running_.load())
	{
		return true;
	}

	if (!driver_ || !driver_->configureInput(pin_, false))
	{
		return false;
	}

	bool raw_value = false;
	if (!driver_->readPin(pin_, raw_value))
	{
		return false;
	}

	updateFromValue(raw_value, true);
	last_raw_value_ = raw_value;
	last_raw_value_valid_ = true;

	running_.store(true);
	worker_ = std::thread(&Mcp23017DownwardSensor::workerLoop, this);
	return true;
}

void Mcp23017DownwardSensor::stop()
{
	if (!running_.exchange(false))
	{
		return;
	}

	edge_cv_.notify_all();

	if (worker_.joinable())
	{
		worker_.join();
	}
}

DownwardReading Mcp23017DownwardSensor::latest() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return latest_reading_;
}

bool Mcp23017DownwardSensor::waitForEdge(const std::chrono::milliseconds timeout)
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

void Mcp23017DownwardSensor::setCallback(DownwardCallback callback)
{
	std::lock_guard<std::mutex> lock(mutex_);
	callback_ = std::move(callback);
}

void Mcp23017DownwardSensor::workerLoop()
{
	const auto poll_period = std::chrono::milliseconds(RobotConfig::MCP23017::POLL_INTERVAL_MS);

	while (running_.load())
	{
		bool raw_value = false;
		if (driver_ && driver_->readPin(pin_, raw_value))
		{
			if (!last_raw_value_valid_ || raw_value != last_raw_value_)
			{
				last_raw_value_ = raw_value;
				last_raw_value_valid_ = true;
				updateFromValue(raw_value, true);
			}
		}

		std::this_thread::sleep_for(poll_period);
	}
}

void Mcp23017DownwardSensor::updateFromValue(const bool sensor_active, const bool notify_edge)
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
		pending_edge_ = notify_edge;
		callback = callback_;
		snapshot = latest_reading_;
	}

	if (notify_edge)
	{
		edge_cv_.notify_all();
	}

	if (callback)
	{
		callback(snapshot);
	}
}
}
