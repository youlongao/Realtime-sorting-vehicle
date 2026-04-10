#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <string>

#include "config.h"
#include "hardware_interfaces.h"

struct gpiod_chip;
struct gpiod_line_request;

namespace Robot
{
class MotorDriver : public IDriveSection
{
public:
	MotorDriver(std::string name,
				unsigned int left_in1_gpio,
				unsigned int left_in2_gpio,
				unsigned int right_in3_gpio,
				unsigned int right_in4_gpio,
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
	bool requestLines();
	void releaseLines();
	bool setLineValue(unsigned int gpio, bool active);
	bool applyMotorCommand(float speed, unsigned int in1_gpio, unsigned int in2_gpio);

	std::string name_;
	std::string chip_path_;
	unsigned int left_in1_gpio_;
	unsigned int left_in2_gpio_;
	unsigned int right_in3_gpio_;
	unsigned int right_in4_gpio_;

	gpiod_chip* chip_{nullptr};
	gpiod_line_request* request_{nullptr};
};
}

#endif
