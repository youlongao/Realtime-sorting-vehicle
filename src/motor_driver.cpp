#include "motor_driver.h"

#include <cmath>
#include <cstring>
#include <utility>

#include "logger.h"
#include "utils.h"

#if defined(__linux__) && defined(CLIMBING_ROBOT_HAS_LIBGPIOD)
#include <cerrno>

#include <gpiod.h>
#endif

namespace Robot
{
namespace
{
std::string FormatErrno(const std::string& prefix)
{
#if defined(__linux__) && defined(CLIMBING_ROBOT_HAS_LIBGPIOD)
	return prefix + ": " + std::strerror(errno);
#else
	return prefix;
#endif
}
}

MotorDriver::MotorDriver(std::string name,
						 std::shared_ptr<Pca9685Driver> pwm_driver,
						 const unsigned int left_pwm_channel,
						 const unsigned int right_pwm_channel,
						 const unsigned int left_in1,
						 const unsigned int left_in2,
						 const unsigned int right_in1,
						 const unsigned int right_in2,
						 std::string chip_path)
	: name_(std::move(name)),
	  chip_path_(std::move(chip_path)),
	  pwm_driver_(std::move(pwm_driver)),
	  left_pwm_channel_(left_pwm_channel),
	  right_pwm_channel_(right_pwm_channel),
	  left_in1_(left_in1),
	  left_in2_(left_in2),
	  right_in1_(right_in1),
	  right_in2_(right_in2)
{
}

MotorDriver::~MotorDriver()
{
	stop();
#if defined(__linux__) && defined(CLIMBING_ROBOT_HAS_LIBGPIOD)
	releaseGpio();
#endif
}

bool MotorDriver::start()
{
	if (!pwm_driver_ || !pwm_driver_->start())
	{
		Logger::error(name_ + " motor driver failed to start PCA9685 PWM output.");
		return false;
	}

#if defined(__linux__) && defined(CLIMBING_ROBOT_HAS_LIBGPIOD)
	return initialiseGpio();
#else
	Logger::warn(name_ + " motor driver GPIO start skipped because libgpiod v2 is unavailable.");
	return false;
#endif
}

void MotorDriver::setSpeed(const float left_speed, const float right_speed)
{
	setNormalizedSpeed(left_speed, right_speed);
}

void MotorDriver::forward(const float speed)
{
	setNormalizedSpeed(std::fabs(speed), std::fabs(speed));
}

void MotorDriver::backward(const float speed)
{
	setNormalizedSpeed(-std::fabs(speed), -std::fabs(speed));
}

void MotorDriver::setNormalizedSpeed(const float left_speed, const float right_speed)
{
	if (!start())
	{
		return;
	}

	applyMotorCommand(clamp(left_speed, -1.0F, 1.0F), left_pwm_channel_, left_in1_, left_in2_);
	applyMotorCommand(clamp(right_speed, -1.0F, 1.0F), right_pwm_channel_, right_in1_, right_in2_);
}

void MotorDriver::stop()
{
	if (pwm_driver_)
	{
		pwm_driver_->disableChannel(static_cast<std::uint8_t>(left_pwm_channel_));
		pwm_driver_->disableChannel(static_cast<std::uint8_t>(right_pwm_channel_));
	}

#if defined(__linux__) && defined(CLIMBING_ROBOT_HAS_LIBGPIOD)
	if (request_ != nullptr)
	{
		gpiod_line_request_set_value(request_, left_in1_, GPIOD_LINE_VALUE_INACTIVE);
		gpiod_line_request_set_value(request_, left_in2_, GPIOD_LINE_VALUE_INACTIVE);
		gpiod_line_request_set_value(request_, right_in1_, GPIOD_LINE_VALUE_INACTIVE);
		gpiod_line_request_set_value(request_, right_in2_, GPIOD_LINE_VALUE_INACTIVE);
	}
#endif
}

void MotorDriver::brake()
{
#if defined(__linux__) && defined(CLIMBING_ROBOT_HAS_LIBGPIOD)
	if (request_ != nullptr)
	{
		gpiod_line_request_set_value(request_, left_in1_, GPIOD_LINE_VALUE_ACTIVE);
		gpiod_line_request_set_value(request_, left_in2_, GPIOD_LINE_VALUE_ACTIVE);
		gpiod_line_request_set_value(request_, right_in1_, GPIOD_LINE_VALUE_ACTIVE);
		gpiod_line_request_set_value(request_, right_in2_, GPIOD_LINE_VALUE_ACTIVE);
	}
#endif

	if (pwm_driver_)
	{
		pwm_driver_->disableChannel(static_cast<std::uint8_t>(left_pwm_channel_));
		pwm_driver_->disableChannel(static_cast<std::uint8_t>(right_pwm_channel_));
	}
}

bool MotorDriver::applyMotorCommand(const float speed,
									const unsigned int pwm_channel,
									const unsigned int in1_offset,
									const unsigned int in2_offset)
{
#if defined(__linux__) && defined(CLIMBING_ROBOT_HAS_LIBGPIOD)
	if (request_ == nullptr || !pwm_driver_)
	{
		return false;
	}

	if (speed > 0.0F)
	{
		gpiod_line_request_set_value(request_, in1_offset, GPIOD_LINE_VALUE_ACTIVE);
		gpiod_line_request_set_value(request_, in2_offset, GPIOD_LINE_VALUE_INACTIVE);
		return pwm_driver_->setDutyCycle(static_cast<std::uint8_t>(pwm_channel), std::fabs(speed));
	}

	if (speed < 0.0F)
	{
		gpiod_line_request_set_value(request_, in1_offset, GPIOD_LINE_VALUE_INACTIVE);
		gpiod_line_request_set_value(request_, in2_offset, GPIOD_LINE_VALUE_ACTIVE);
		return pwm_driver_->setDutyCycle(static_cast<std::uint8_t>(pwm_channel), std::fabs(speed));
	}

	gpiod_line_request_set_value(request_, in1_offset, GPIOD_LINE_VALUE_INACTIVE);
	gpiod_line_request_set_value(request_, in2_offset, GPIOD_LINE_VALUE_INACTIVE);
	return pwm_driver_->disableChannel(static_cast<std::uint8_t>(pwm_channel));
#else
	(void)speed;
	(void)pwm_channel;
	(void)in1_offset;
	(void)in2_offset;
	return false;
#endif
}

#if defined(__linux__) && defined(CLIMBING_ROBOT_HAS_LIBGPIOD)
bool MotorDriver::initialiseGpio()
{
	if (request_ != nullptr)
	{
		return true;
	}

	chip_ = gpiod_chip_open(chip_path_.c_str());
	if (chip_ == nullptr)
	{
		Logger::error(FormatErrno("Failed to open gpiochip for motor driver " + name_));
		return false;
	}

	auto* settings = gpiod_line_settings_new();
	auto* config = gpiod_line_config_new();
	auto* request_config = gpiod_request_config_new();

	if (settings == nullptr || config == nullptr || request_config == nullptr)
	{
		Logger::error("Failed to allocate libgpiod objects for motor driver " + name_);
		gpiod_line_settings_free(settings);
		gpiod_line_config_free(config);
		gpiod_request_config_free(request_config);
		releaseGpio();
		return false;
	}

	gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
	gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);
	gpiod_line_settings_set_drive(settings, GPIOD_LINE_DRIVE_PUSH_PULL);

	const unsigned int offsets[4] = {left_in1_, left_in2_, right_in1_, right_in2_};
	gpiod_line_config_add_line_settings(config, offsets, 4, settings);
	gpiod_request_config_set_consumer(request_config, name_.c_str());

	request_ = gpiod_chip_request_lines(chip_, request_config, config);

	gpiod_line_settings_free(settings);
	gpiod_line_config_free(config);
	gpiod_request_config_free(request_config);

	if (request_ == nullptr)
	{
		Logger::error(FormatErrno("Failed to request direction GPIOs for motor driver " + name_));
		releaseGpio();
		return false;
	}

	return true;
}

void MotorDriver::releaseGpio()
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
#endif
}
