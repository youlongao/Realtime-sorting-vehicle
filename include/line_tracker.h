#ifndef LINE_TRACKER_H
#define LINE_TRACKER_H

#include "share_types.h"
#include <cstdint>

class LineTracker {
public:
	LineTracker(int leftpin, int centerpin, int rightpin);
	~LineTracker();

	bool initialize();
	
	void registerCallback(LineCallback callback);	// callback function when a change in line direction is detected

private:
	int leftpin, centerpin, rightpin;
	LineCallback m_callback;

	// when any sensor pin level chages, gpio is invoked and get current object pointer via userdata
	static void pinInterruptWrapper(int gpio, int level, uint32_t tick, void* userdata);

	// actual direction judgement logic function
	void evaluateDirection();
};

#endif