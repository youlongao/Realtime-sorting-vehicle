#include "rear_support_module.h"

#include <chrono>
#include <string>
#include <utility>

#include "logger.h"

namespace Robot
{
RearSupportModule::RearSupportModule(ILinearAxis* rear_slide_axis,
									 ILinearAxis* rear_lift_axis,
									 std::function<bool()> rear_support_confirmed)
	: rear_slide_axis_(rear_slide_axis),
	  rear_lift_axis_(rear_lift_axis),
	  rear_support_confirmed_(std::move(rear_support_confirmed))
{
}

// Assist in adjusting the overall machine attitude during the mid-section transfer phase
// The rear slide remains stationary, while the second lifting module continues to rise
// Used in conjunction with the first lifting module to help the middle wheel assembly complete the transfer
void RearSupportModule::assistMiddleTransfer()
{
	// During the mid-section transfer,the rear slide should maintain 
	// its current position to prevent additional displacement of the rear support structure
	if (rear_slide_axis_ != nullptr)
	{
		rear_slide_axis_->holdPosition();
	}

	// The second lifting module rises to participate in the overall machine attitude adjustment
	if (rear_lift_axis_ != nullptr)
	{
		rear_lift_axis_->moveNormalized(RobotConfig::Motion::BODY_LIFT_SPEED);
	}
}

// During the later transfer phase, the rear support wheel is transferred to the new step surface
bool RearSupportModule::transferSupportToStep()
{
	// The rear slide is pushed forward, propelling the rear support structure closer to the new support surface
	if (rear_slide_axis_ != nullptr)
	{
		rear_slide_axis_->moveNormalized(RobotConfig::Motion::CREEP_SPEED);
	}

	// The second lifting module is lowered, 
	// allowing the rear support wheel to gradually descend onto the new step surface
	if (rear_lift_axis_ != nullptr)
	{
		rear_lift_axis_->moveNormalized(RobotConfig::Motion::BODY_LOWER_SPEED);
	}

	// If the subsequent support confirms a pullback, and the rear wheel has already landed
	// stop action to indicate that rear section transfer is finished
	if (rear_support_confirmed_ && rear_support_confirmed_())
	{
		stabilizeSupport();
		return true;
	}

	// The latter part has not yet been completed; the current process continues
	return false;
}

bool RearSupportModule::moveSlideBackwardUntilLimit()
{
	if (rear_slide_axis_ == nullptr)
	{
		return true;
	}

	const auto axis_state = rear_slide_axis_->getAxisState();
	if (axis_state.at_lower_limit)
	{
		rear_slide_axis_->holdPosition();
		Logger::info("Rear slider reached the rear lower limit.");
		return true;
	}

	logRearSlideBackStatus(axis_state);
	rear_slide_axis_->moveNormalized(RobotConfig::Motion::SLIDER_HOME_SPEED);
	return false;
}

bool RearSupportModule::moveSlideForwardUntilLimit()
{
	if (rear_slide_axis_ == nullptr)
	{
		return true;
	}

	const auto axis_state = rear_slide_axis_->getAxisState();
	if (axis_state.at_upper_limit)
	{
		rear_slide_axis_->holdPosition();
		Logger::info("Rear slider reached the front upper limit.");
		return true;
	}

	rear_slide_axis_->moveNormalized(RobotConfig::Motion::CREEP_SPEED);
	return false;
}

bool RearSupportModule::liftRearUntilUpperLimit()
{
	if (rear_lift_axis_ == nullptr)
	{
		return true;
	}

	const auto axis_state = rear_lift_axis_->getAxisState();
	if (axis_state.at_upper_limit)
	{
		rear_lift_axis_->holdPosition();
		Logger::info("Rear lift reached the upper limit.");
		return true;
	}

	rear_lift_axis_->moveNormalized(RobotConfig::Motion::BODY_LIFT_SPEED);
	return false;
}

bool RearSupportModule::lowerRearUntilLowerLimit()
{
	if (rear_lift_axis_ == nullptr)
	{
		return true;
	}

	const auto axis_state = rear_lift_axis_->getAxisState();
	if (axis_state.at_lower_limit)
	{
		rear_lift_axis_->holdPosition();
		Logger::info("Rear lift reached the lower limit.");
		return true;
	}

	logRearLiftLowerStatus(axis_state);
	rear_lift_axis_->moveNormalized(RobotConfig::Motion::BODY_LOWER_SPEED);
	return false;
}

bool RearSupportModule::isSupportConfirmed() const
{
	return rear_support_confirmed_ && rear_support_confirmed_();
}

void RearSupportModule::logRearSlideBackStatus(const AxisState& axis_state)
{
	const auto now = SteadyClock::now();
	if (last_rear_slide_back_log_time_ != Timestamp{} &&
		now - last_rear_slide_back_log_time_ < std::chrono::seconds(1))
	{
		return;
	}

	last_rear_slide_back_log_time_ = now;
	Logger::info(
		std::string("RearSliderBack waiting: lower_limit=") +
		(axis_state.at_lower_limit ? "1" : "0") +
		", upper_limit=" +
		(axis_state.at_upper_limit ? "1" : "0"));
}

void RearSupportModule::logRearLiftLowerStatus(const AxisState& axis_state)
{
	const auto now = SteadyClock::now();
	if (last_rear_lift_lower_log_time_ != Timestamp{} &&
		now - last_rear_lift_lower_log_time_ < std::chrono::seconds(1))
	{
		return;
	}

	last_rear_lift_lower_log_time_ = now;
	Logger::info(
		std::string("MiddleClimb rear lift lowering: lower_limit=") +
		(axis_state.at_lower_limit ? "1" : "0") +
		", upper_limit=" +
		(axis_state.at_upper_limit ? "1" : "0"));
}

// Maintain the current position of the rear support structure
void RearSupportModule::stabilizeSupport()
{
	// Keep the current position of the rear slide
	if (rear_slide_axis_ != nullptr)
	{
		rear_slide_axis_->holdPosition();
	}

	// Maintain the current position of the second lifting module
	if (rear_lift_axis_ != nullptr)
	{
		rear_lift_axis_->holdPosition();
	}
}
}
