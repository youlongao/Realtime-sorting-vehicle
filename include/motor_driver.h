#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <memory>
#include <string>

#include "hardware_interfaces.h"
#include "pca9685_driver.h"

struct gpiod_chip;
struct gpiod_line_request;

namespace Robot
{
class MotorDriver : public IDriveSection
{
public:
	MotorDriver(std::string name,
				std::shared_ptr<Pca9685Driver> pwm_driver,
				unsigned int left_pwm_channel,
				unsigned int right_pwm_channel,
				unsigned int left_in1,
				unsigned int left_in2,
				unsigned int right_in1,
				unsigned int right_in2,
				std::string chip_path = RobotConfig::Platform::GPIO_CHIP);
	~MotorDriver() override;

	bool start();

	void setSpeed(float left_speed, float right_speed);
	void forward(float speed);
	void backward(float speed);
	void setNormalizedSpeed(float left_speed, float right_speed) override;
	void stop() override;
	void brake() override;

private:
	bool applyMotorCommand(float speed,
						   unsigned int pwm_channel,
						   unsigned int in1_offset,
						   unsigned int in2_offset);

#if defined(__linux__) && defined(CLIMBING_ROBOT_HAS_LIBGPIOD)
	bool initialiseGpio();
	void releaseGpio();
#endif

	std::string name_;
	std::string chip_path_;
	std::shared_ptr<Pca9685Driver> pwm_driver_;
	unsigned int left_pwm_channel_;
	unsigned int right_pwm_channel_;
	unsigned int left_in1_;
	unsigned int left_in2_;
	unsigned int right_in1_;
	unsigned int right_in2_;

#if defined(__linux__) && defined(CLIMBING_ROBOT_HAS_LIBGPIOD)
	gpiod_chip* chip_{nullptr};
	gpiod_line_request* request_{nullptr};
#endif
};
}

#endif
