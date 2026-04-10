#ifndef LINEAR_ACTUATOR_H
#define LINEAR_ACTUATOR_H

#include <mutex>
#include <string>

#include "config.h"
#include "hardware_interfaces.h"

struct gpiod_chip;
struct gpiod_line_request;

namespace Robot
{
class LinearActuator : public ILinearAxis
{
public:
	LinearActuator(unsigned int forward_gpio,
				   unsigned int reverse_gpio,
				   ILimitSwitch* upper_limit = nullptr,
				   ILimitSwitch* lower_limit = nullptr,
				   float max_travel_m = RobotConfig::Geometry::BODY_LIFT_MAX_TRAVEL_M,
				   std::string chip_path = RobotConfig::Platform::GPIO_CHIP);

	bool start();
	void extend(float speed = RobotConfig::Motion::BODY_LIFT_SPEED);
	void retract(float speed = -RobotConfig::Motion::BODY_LOWER_SPEED);
	bool moveToPosition(float target_position_m);
	void stop() override;
	bool isAtLimit() const;

	void moveNormalized(float command) override;
	void holdPosition() override;
	AxisState getAxisState() const override;

private:
	bool requestLines();
	void releaseLines();
	bool setLineValue(unsigned int gpio, bool active);
	void updateCachedState() const;

	std::string chip_path_;
	unsigned int forward_gpio_;
	unsigned int reverse_gpio_;
	ILimitSwitch* upper_limit_;
	ILimitSwitch* lower_limit_;
	float max_travel_m_;

	mutable std::mutex mutex_;
	mutable AxisState axis_state_;

	gpiod_chip* chip_{nullptr};
	gpiod_line_request* request_{nullptr};
};
}

#endif
