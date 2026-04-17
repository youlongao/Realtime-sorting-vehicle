#ifndef REAR_SUPPORT_MODULE_H
#define REAR_SUPPORT_MODULE_H

#include <functional>

#include "config.h"
#include "hardware_interfaces.h"

namespace Robot
{
class RearSupportModule
{
public:
	/*
	rear_slide_axis: Linear actuator interface of the rear sliding module
	rear_lift_axis: Linear actuator interface of the second lifting module
	rear_support_confirmed: A callback function is used to confirm 
							whether the subsequent support wheel has truly landed on the new step surface
	*/
	RearSupportModule(ILinearAxis* rear_slide_axis = nullptr,
					  ILinearAxis* rear_lift_axis = nullptr,
					  std::function<bool()> rear_support_confirmed = {});

	// Assist in adjusting the overall machine attitude during the mid-section transfer phase
	void assistMiddleTransfer();

	// The rear support wheel is moved towards the new step surface
	bool transferSupportToStep();

	// Move rear slider backward to the rear limit before lifting the front section
	bool moveSlideBackwardUntilLimit();

	// Move rear slider forward to the front limit before lifting the rear section
	bool moveSlideForwardUntilLimit();

	// Lift the rear section until the rear lift upper limit is reached
	bool liftRearUntilUpperLimit();

	// Lower the rear lift until the rear lower limit is reached
	bool lowerRearUntilLowerLimit();

	// True after the rear downward sensor confirms landing
	bool isSupportConfirmed() const;

	// Stop the movement of the rear support mechanism and maintain its current position
	void stabilizeSupport();

private:
	void logRearSlideBackStatus(const AxisState& axis_state);
	void logRearLiftLowerStatus(const AxisState& axis_state);

	ILinearAxis* rear_slide_axis_;
	ILinearAxis* rear_lift_axis_;
	std::function<bool()> rear_support_confirmed_;
	Timestamp last_rear_slide_back_log_time_{};
	Timestamp last_rear_lift_lower_log_time_{};
};
}

#endif
