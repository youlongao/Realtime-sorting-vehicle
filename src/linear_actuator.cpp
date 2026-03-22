#include "linear_actuator.h"

#include <cmath>
#include <utility>

#include "logger.h"
#include "utils.h"

namespace Robot
{
LinearActuator::LinearActuator(std::shared_ptr<Pca9685Driver> pwm_driver,
							   const unsigned int forward_pwm_channel,
							   const unsigned int reverse_pwm_channel,
							   IEncoderFeedback* encoder,
							   ILimitSwitch* upper_limit,
							   ILimitSwitch* lower_limit,
							   const float max_travel_m)
	: pwm_driver_(std::move(pwm_driver)),
	  forward_pwm_channel_(forward_pwm_channel),
	  reverse_pwm_channel_(reverse_pwm_channel),
	  encoder_(encoder),
	  upper_limit_(upper_limit),
	  lower_limit_(lower_limit),
	  max_travel_m_(max_travel_m)
{
}

bool LinearActuator::start()
{
	if (!pwm_driver_ || !pwm_driver_->start())
	{
		Logger::error("Linear actuator failed to start PCA9685 PWM output.");
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

	pwm_driver_->setDutyCycle(static_cast<std::uint8_t>(forward_pwm_channel_), clamp(std::fabs(speed), 0.0F, 1.0F));
	pwm_driver_->disableChannel(static_cast<std::uint8_t>(reverse_pwm_channel_));

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

	pwm_driver_->disableChannel(static_cast<std::uint8_t>(forward_pwm_channel_));
	pwm_driver_->setDutyCycle(static_cast<std::uint8_t>(reverse_pwm_channel_), clamp(std::fabs(speed), 0.0F, 1.0F));

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
	if (pwm_driver_ != nullptr)
	{
		pwm_driver_->disableChannel(static_cast<std::uint8_t>(forward_pwm_channel_));
		pwm_driver_->disableChannel(static_cast<std::uint8_t>(reverse_pwm_channel_));
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

void LinearActuator::updateCachedState() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (encoder_ != nullptr)
	{
		axis_state_.position_m = clamp(encoder_->getDistance(), 0.0F, max_travel_m_);
		axis_state_.homed = true;
	}

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
