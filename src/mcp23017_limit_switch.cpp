#include "mcp23017_limit_switch.h"

#include <chrono>
#include <utility>

namespace Robot
{
Mcp23017LimitSwitch::Mcp23017LimitSwitch(std::shared_ptr<Mcp23017Driver> driver,
										 const std::uint8_t pin,
										 const LimitRole role,
										 const bool active_low)
	: driver_(std::move(driver)),
	  pin_(pin),
	  role_(role),
	  active_low_(active_low)
{
}

Mcp23017LimitSwitch::~Mcp23017LimitSwitch()
{
	stop();
}

bool Mcp23017LimitSwitch::start()
{
	if (running_.load())
	{
		return true;
	}

	if (!driver_ || !driver_->configureInput(pin_, active_low_))
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
	worker_ = std::thread(&Mcp23017LimitSwitch::workerLoop, this);
	return true;
}

void Mcp23017LimitSwitch::stop()
{
	if (!running_.exchange(false))
	{
		return;
	}

	trigger_cv_.notify_all();

	if (worker_.joinable())
	{
		worker_.join();
	}
}

bool Mcp23017LimitSwitch::isTriggered() const
{
	return latestState().triggered;
}

bool Mcp23017LimitSwitch::isUpperLimit() const
{
	return role_ == LimitRole::Upper && isTriggered();
}

bool Mcp23017LimitSwitch::isLowerLimit() const
{
	return role_ == LimitRole::Lower && isTriggered();
}

LimitSwitchState Mcp23017LimitSwitch::latestState() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return state_;
}

bool Mcp23017LimitSwitch::waitForTrigger(const std::chrono::milliseconds timeout)
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

void Mcp23017LimitSwitch::setCallback(LimitSwitchCallback callback)
{
	std::lock_guard<std::mutex> lock(mutex_);
	callback_ = std::move(callback);
}

void Mcp23017LimitSwitch::workerLoop()
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

void Mcp23017LimitSwitch::updateFromValue(const bool raw_value, const bool notify_edge)
{
	LimitSwitchCallback callback;
	LimitSwitchState snapshot;

	{
		std::lock_guard<std::mutex> lock(mutex_);
		state_.triggered = active_low_ ? !raw_value : raw_value;
		state_.timestamp = SteadyClock::now();
		state_.valid = true;
		pending_trigger_ = state_.triggered && notify_edge;
		callback = callback_;
		snapshot = state_;
	}

	if (notify_edge)
	{
		trigger_cv_.notify_all();
	}

	if (callback)
	{
		callback(snapshot);
	}
}
}
