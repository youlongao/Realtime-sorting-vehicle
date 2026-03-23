#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <memory>

#include "config.h"
#include "pca9685_driver.h"

namespace Robot
{
class ServoDriver
{
public:
	ServoDriver(std::shared_ptr<Pca9685Driver> pwm_driver,
				unsigned int pwm_channel,
				float min_angle_deg = RobotConfig::Servo::MIN_ANGLE_DEG,
				float max_angle_deg = RobotConfig::Servo::MAX_ANGLE_DEG,
				float min_pulse_width_us = RobotConfig::Servo::MIN_PULSE_WIDTH_US,
				float max_pulse_width_us = RobotConfig::Servo::MAX_PULSE_WIDTH_US,
				float safe_angle_deg = RobotConfig::Servo::SAFE_ANGLE_DEG);

	bool start();
	void setAngle(float angle_deg);
	void setPulseWidth(float pulse_width_us);
	void moveToSafePosition();

private:
	std::shared_ptr<Pca9685Driver> pwm_driver_;
	unsigned int pwm_channel_;
	float min_angle_deg_;
	float max_angle_deg_;
	float min_pulse_width_us_;
	float max_pulse_width_us_;
	float safe_angle_deg_;
};
}

#endif
