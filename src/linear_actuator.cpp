#include "linear_actuator.h"

#include <cmath>
#include <cstring>
#include <utility>

#include "logger.h"
#include "utils.h"

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

LinearActuator::LinearActuator(const unsigned int forward_gpio,
							   const unsigned int reverse_gpio,
							   ILimitSwitch* upper_limit,
							   ILimitSwitch* lower_limit,
							   const float max_travel_m,
							   std::string chip_path)
	: chip_path_(std::move(chip_path)),
	  forward_gpio_(forward_gpio),
	  reverse_gpio_(reverse_gpio),
	  upper_limit_(upper_limit),
	  lower_limit_(lower_limit),
	  max_travel_m_(max_travel_m)
{
}

bool LinearActuator::start()
{
	if (request_ == nullptr && !requestLines())
	{
		Logger::error("Linear actuator failed to request GPIO outputs.");
		return false;
	}

	updateCachedState();
	return true;
}

void LinearActuator::extend(float speed)
{
	if (!start())
	{
		return;
	}

	updateCachedState();
	if (upper_limit_ != nullptr && upper_limit_->latestState().triggered)
	{
		stop();
		return;
	}

	(void)setLineValue(forward_gpio_, clamp(std::fabs(speed), 0.0F, 1.0F) > 0.0F);
	(void)setLineValue(reverse_gpio_, false);

	std::lock_guard<std::mutex> lock(mutex_);
	axis_state_.in_motion = true;
	axis_state_.timestamp = SteadyClock::now();
}

void LinearActuator::retract(float speed)
{
	if (!start())
	{
		return;
	}

	updateCachedState();
	if (lower_limit_ != nullptr && lower_limit_->latestState().triggered)
	{
		stop();
		return;
	}

	(void)setLineValue(forward_gpio_, false);
	(void)setLineValue(reverse_gpio_, clamp(std::fabs(speed), 0.0F, 1.0F) > 0.0F);

	std::lock_guard<std::mutex> lock(mutex_);
	axis_state_.in_motion = true;
	axis_state_.timestamp = SteadyClock::now();
}

bool LinearActuator::moveToPosition(const float target_position_m)
{
	updateCachedState();
	const auto axis_state = getAxisState();
	const float clamped_target = clamp(target_position_m, 0.0F, max_travel_m_);
	const float error = clamped_target - axis_state.position_m;

	if (std::fabs(error) <= RobotConfig::Geometry::POSITION_TOLERANCE_M)
	{
		holdPosition();
		return true;
	}

	if (error > 0.0F)
	{
		extend(RobotConfig::Motion::BODY_LIFT_SPEED);
		return false;
	}

	retract(-RobotConfig::Motion::BODY_LOWER_SPEED);
	return false;
}

void LinearActuator::stop()
{
	if (request_ != nullptr)
	{
		(void)setLineValue(forward_gpio_, false);
		(void)setLineValue(reverse_gpio_, false);
	}

	std::lock_guard<std::mutex> lock(mutex_);
	axis_state_.in_motion = false;
	axis_state_.timestamp = SteadyClock::now();
}

bool LinearActuator::isAtLimit() const
{
	const auto state = getAxisState();
	return state.at_upper_limit || state.at_lower_limit;
}

void LinearActuator::moveNormalized(const float command)
{
	if (command > 0.0F)
	{
		extend(command);
		return;
	}

	if (command < 0.0F)
	{
		retract(-command);
		return;
	}

	stop();
}

void LinearActuator::holdPosition()
{
	stop();
}

AxisState LinearActuator::getAxisState() const
{
	updateCachedState();

	std::lock_guard<std::mutex> lock(mutex_);
	return axis_state_;
}

bool LinearActuator::requestLines()
{
	chip_ = gpiod_chip_open(chip_path_.c_str());
	if (chip_ == nullptr)
	{
		Logger::error(FormatErrno("Failed to open gpiochip for linear actuator"));
		return false;
	}

	auto* settings = gpiod_line_settings_new();
	auto* line_config = gpiod_line_config_new();
	auto* request_config = gpiod_request_config_new();

	if (settings == nullptr || line_config == nullptr || request_config == nullptr)
	{
		Logger::error("Failed to allocate libgpiod objects for linear actuator.");
		gpiod_line_settings_free(settings);
		gpiod_line_config_free(line_config);
		gpiod_request_config_free(request_config);
		releaseLines();
		return false;
	}

	gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
	gpiod_line_settings_set_drive(settings, GPIOD_LINE_DRIVE_PUSH_PULL);
	gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);
	gpiod_request_config_set_consumer(request_config, "linear-actuator");

	const unsigned int offsets[2] = {forward_gpio_, reverse_gpio_};
	gpiod_line_config_add_line_settings(line_config, offsets, 2, settings);
	request_ = gpiod_chip_request_lines(chip_, request_config, line_config);

	gpiod_line_settings_free(settings);
	gpiod_line_config_free(line_config);
	gpiod_request_config_free(request_config);

	if (request_ == nullptr)
	{
		Logger::error(FormatErrno("Failed to request GPIO lines for linear actuator"));
		releaseLines();
		return false;
	}

	stop();
	return true;
}

void LinearActuator::releaseLines()
{
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

bool LinearActuator::setLineValue(const unsigned int gpio, const bool active)
{
	if (request_ == nullptr)
	{
		return false;
	}

	return gpiod_line_request_set_value(
			   request_,
			   gpio,
			   active ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE) >= 0;
}

void LinearActuator::updateCachedState() const
{
	std::lock_guard<std::mutex> lock(mutex_);

	if (upper_limit_ != nullptr)
	{
		axis_state_.at_upper_limit = upper_limit_->latestState().triggered;
	}

	if (lower_limit_ != nullptr)
	{
		axis_state_.at_lower_limit = lower_limit_->latestState().triggered;
		if (axis_state_.at_lower_limit)
		{
			axis_state_.position_m = 0.0F;
			axis_state_.homed = true;
		}
	}

	axis_state_.timestamp = SteadyClock::now();
}
}
