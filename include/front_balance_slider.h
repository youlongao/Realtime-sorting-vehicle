#ifndef FRONT_BALANCE_SLIDER_H
#define FRONT_BALANCE_SLIDER_H

#include <mutex>

#include "config.h"
#include "hardware_interfaces.h"

namespace Robot
{
class FrontBalanceSlider
{
public:
	// The linear actuator interface used for binding the front slide
	explicit FrontBalanceSlider(ILinearAxis& slider_axis);

	void updateExtension(float body_extension_m);	// update extension of the slide before update
	float getExtension() const;		// get current extension of the slide
	bool isPositionValid() const;	// Determine if the current position of the front slide is valid
	void home();

private:
	ILinearAxis& slider_axis_;

	mutable std::mutex mutex_;	// Protect the position and effectiveness of the slide

	// The current extension of the front slide is initially set to the offset of the slide origin.
	float slider_extension_m_{RobotConfig::Geometry::SLIDER_HOME_OFFSET_M};		

	bool position_valid_{false};	// marking the current slide position whether valid or not
};
}

#endif
