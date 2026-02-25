#include "line_tracker.h"
#include <gpio.h>
#include <iostream>

LineTracker::LineTracker(int leftpin, int centerpin, int rightpin)
	: leftpin(leftpin), centerpin(centerpin), rightpin(rightpin){ }

LineTracker::~LineTracker()
{
	gpioSetAlertFuncEx(leftpin, nullptr, nullptr);
	gpioSetAlertFuncEx(centerpin, nullptr, nullptr);
	gpioSetAlertFuncEx(rightpin, nullptr, nullptr);
}

bool LineTracker::initialize()
{
	std::cout << "Initializing line tracker pins..." << std::endl;

	gpioSetMode(leftPin, PI_INPUT);
	gpioSetMode(centerPin, PI_INPUT);
	gpioSetMode(rightPin, PI_INPUT);

	// regist edge-triggered interrupts for all three pins
	gpioSetAlertFuncEx(leftpin, pinInterruptWrapper, this);
	gpioSetAlertFuncEx(centerpin, pinInterruptWrapper, this);
	gpioSetAlertFuncEx(rightpin, pinInterruptWrapper, this);

	return true;
}

void LineTracker::registerCallback(LineCallback callback)
{
	m_callback = std::move(callback);
}

void LineTracker::pinInterruptWrapper(int gpio, int level, uint32_t tick, void* userdata)
{
	if (level != 0 && level != 1)
		return;

	LineTracker* tracker = static_cast<LineTracker*>(userdata);
	
	if (tracker) {
		// if any pin changes direction, reassess the orientation
		tracker->evaluateDirection();
	}
}

void LineTracker::evaluateDirection()
{
	// the current level state is read only when hardware interrupt is triggered
	int leftState = gpioRead(leftpin);
	int centerState = gpioRead(centerpin);
	int rightState = gpioRead(rightpin);

	int direction = 2;	// line lose

	if (centerState == 1) {
		direction = 0;	// center
	}
	else if (leftState == 1) {
		direction = -1;	// left, corrected to the right
	}
	else if (rightState == 1) {
		direction = 1;	// right, corrected to the left
	}

	if (m_callback) {
		m_callback(direction);
	}
}