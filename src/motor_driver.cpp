#include "motor_driver.h"

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

MotorDriver::MotorDriver(std::string name,
						 const unsigned int left_in1_gpio,
						 const unsigned int left_in2_gpio,
						 const unsigned int right_in3_gpio,
						 const unsigned int right_in4_gpio,
						 std::string chip_path)
	: name_(std::move(name)),
	  chip_path_(std::move(chip_path)),
	  left_in1_gpio_(left_in1_gpio),
	  left_in2_gpio_(left_in2_gpio),
	  right_in3_gpio_(right_in3_gpio),
	  right_in4_gpio_(right_in4_gpio)
{
}

MotorDriver::~MotorDriver()
{
	stop();
	releaseLines();
}

bool MotorDriver::start()
{
	if (request_ != nullptr)
	{
		return true;
	}

	return requestLines();
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

	(void)applyMotorCommand(clamp(left_speed, -1.0F, 1.0F), left_in1_gpio_, left_in2_gpio_);
	(void)applyMotorCommand(clamp(right_speed, -1.0F, 1.0F), right_in3_gpio_, right_in4_gpio_);
}

void MotorDriver::stop()
{
	if (request_ == nullptr)
	{
		return;
	}

	(void)setLineValue(left_in1_gpio_, false);
	(void)setLineValue(left_in2_gpio_, false);
	(void)setLineValue(right_in3_gpio_, false);
	(void)setLineValue(right_in4_gpio_, false);
}

void MotorDriver::brake()
{
	if (!start())
	{
		return;
	}

	(void)setLineValue(left_in1_gpio_, true);
	(void)setLineValue(left_in2_gpio_, true);
	(void)setLineValue(right_in3_gpio_, true);
	(void)setLineValue(right_in4_gpio_, true);
}

bool MotorDriver::requestLines()
{
	chip_ = gpiod_chip_open(chip_path_.c_str());
	if (chip_ == nullptr)
	{
		Logger::error(FormatErrno(name_ + " failed to open gpiochip for wheel driver"));
		return false;
	}

	auto* settings = gpiod_line_settings_new();
	auto* line_config = gpiod_line_config_new();
	auto* request_config = gpiod_request_config_new();

	if (settings == nullptr || line_config == nullptr || request_config == nullptr)
	{
		Logger::error(name_ + " failed to allocate libgpiod output objects.");
		gpiod_line_settings_free(settings);
		gpiod_line_config_free(line_config);
		gpiod_request_config_free(request_config);
		releaseLines();
		return false;
	}

	gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
	gpiod_line_settings_set_drive(settings, GPIOD_LINE_DRIVE_PUSH_PULL);
	gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);
	gpiod_request_config_set_consumer(request_config, name_.c_str());

	const unsigned int offsets[4] = {
		left_in1_gpio_,
		left_in2_gpio_,
		right_in3_gpio_,
		right_in4_gpio_};
	gpiod_line_config_add_line_settings(line_config, offsets, 4, settings);
	request_ = gpiod_chip_request_lines(chip_, request_config, line_config);

	gpiod_line_settings_free(settings);
	gpiod_line_config_free(line_config);
	gpiod_request_config_free(request_config);

	if (request_ == nullptr)
	{
		Logger::error(FormatErrno(name_ + " failed to request wheel driver GPIO lines"));
		releaseLines();
		return false;
	}

	stop();
	return true;
}

void MotorDriver::releaseLines()
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

bool MotorDriver::setLineValue(const unsigned int gpio, const bool active)
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

bool MotorDriver::applyMotorCommand(const float speed,
									const unsigned int in1_gpio,
									const unsigned int in2_gpio)
{
	if (request_ == nullptr)
	{
		return false;
	}

	if (speed > 0.0F)
	{
		return setLineValue(in1_gpio, true) && setLineValue(in2_gpio, false);
	}

	if (speed < 0.0F)
	{
		return setLineValue(in1_gpio, false) && setLineValue(in2_gpio, true);
	}

	return setLineValue(in1_gpio, false) && setLineValue(in2_gpio, false);
}
}
