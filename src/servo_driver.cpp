#include "servo_driver.h"

#include <utility>

#include "utils.h"

namespace Robot
{
ServoDriver::ServoDriver(std::shared_ptr<Pca9685Driver> pwm_driver,
						 const unsigned int pwm_channel,
						 const float min_angle_deg,
						 const float max_angle_deg,
						 const float min_pulse_width_us,
						 const float max_pulse_width_us,
						 const float safe_angle_deg)
	: pwm_driver_(std::move(pwm_driver)),
	  pwm_channel_(pwm_channel),
	  min_angle_deg_(min_angle_deg),
	  max_angle_deg_(max_angle_deg),
	  min_pulse_width_us_(min_pulse_width_us),
	  max_pulse_width_us_(max_pulse_width_us),
	  safe_angle_deg_(safe_angle_deg)
{
}

bool ServoDriver::start()
{
	return pwm_driver_ && pwm_driver_->start();
}

void ServoDriver::setAngle(const float angle_deg)
{
	if (!start())
	{
		return;
	}

	const float clamped_angle = clamp(angle_deg, min_angle_deg_, max_angle_deg_);
	const float normalized = (clamped_angle - min_angle_deg_) / (max_angle_deg_ - min_angle_deg_);
	const float pulse_width =
		min_pulse_width_us_ + (normalized * (max_pulse_width_us_ - min_pulse_width_us_));
	setPulseWidth(pulse_width);
}

void ServoDriver::setPulseWidth(const float pulse_width_us)
{
	if (!start())
	{
		return;
	}

	pwm_driver_->setPulseWidthUs(
		static_cast<std::uint8_t>(pwm_channel_),
		clamp(pulse_width_us, min_pulse_width_us_, max_pulse_width_us_));
}

void ServoDriver::moveToSafePosition()
{
	setAngle(safe_angle_deg_);
}
}
