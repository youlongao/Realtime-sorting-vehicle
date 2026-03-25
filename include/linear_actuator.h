#ifndef LINEAR_ACTUATOR_H
#define LINEAR_ACTUATOR_H

#include <memory>
#include <mutex>

#include "config.h"
#include "hardware_interfaces.h"
#include "pca9685_driver.h"

namespace Robot
{
class LinearActuator : public ILinearAxis
{
public:
	LinearActuator(std::shared_ptr<Pca9685Driver> pwm_driver,
				   unsigned int forward_pwm_channel,
				   unsigned int reverse_pwm_channel,
				   ILimitSwitch* upper_limit = nullptr,
				   ILimitSwitch* lower_limit = nullptr,
				   float max_travel_m = RobotConfig::Geometry::BODY_LIFT_MAX_TRAVEL_M);

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
	void updateCachedState() const;

	std::shared_ptr<Pca9685Driver> pwm_driver_;
	unsigned int forward_pwm_channel_;
	unsigned int reverse_pwm_channel_;
	ILimitSwitch* upper_limit_;
	ILimitSwitch* lower_limit_;
	float max_travel_m_;

	mutable std::mutex mutex_;
	mutable AxisState axis_state_;
};
}

#endif
