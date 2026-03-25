#include "front_balance_slider.h"

#include "logger.h"
#include "utils.h"

namespace Robot
{
FrontBalanceSlider::FrontBalanceSlider(ILinearAxis& slider_axis)
	: slider_axis_(slider_axis)
{
}

void FrontBalanceSlider::updateExtension(const float body_extension_m)
{
	std::lock_guard<std::mutex> lock(mutex_);	// avoid multiple threads modifying the state simultaneously
	slider_extension_m_ = clamp(
		body_extension_m,
		RobotConfig::Geometry::SLIDER_HOME_OFFSET_M,	// minimum extension
		RobotConfig::Geometry::SLIDER_MAX_TRAVEL_M);	// maximum extension

	const auto axis_state = slider_axis_.getAxisState();	// get slide axis state

	// The current slide position is considered valid 
	// only if the slide has returned to zero and the current position is still within the allowed range
	position_valid_ =
		axis_state.homed &&
		slider_extension_m_ >= RobotConfig::Geometry::SLIDER_HOME_OFFSET_M &&
		slider_extension_m_ <= RobotConfig::Geometry::SLIDER_MAX_TRAVEL_M;
}

// get the current extension amount of the front slide
float FrontBalanceSlider::getExtension() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return slider_extension_m_;
}

// judge current position is valid or not
bool FrontBalanceSlider::isPositionValid() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return position_valid_;
}

// control the front slide to perform the homing action
void FrontBalanceSlider::home()
{
	// get front slide current state 
	const auto axis_state = slider_axis_.getAxisState();

	// if front slide has touched lower limit, homing action finished
	if (axis_state.at_lower_limit)
	{
		// stop slide moving
		slider_axis_.stop();

		// update internal states: the current position is set as origin, the position is valid 
		std::lock_guard<std::mutex> lock(mutex_);
		slider_extension_m_ = RobotConfig::Geometry::SLIDER_HOME_OFFSET_M;
		position_valid_ = true;

		// print logger to indicate that homing action finished
		Logger::info("Front balance slider homed.");
		return;
	}

	// if the lower limit has not been touched, continue move towards the origin at the home speed
	slider_axis_.moveNormalized(RobotConfig::Motion::SLIDER_HOME_SPEED);
}
}
