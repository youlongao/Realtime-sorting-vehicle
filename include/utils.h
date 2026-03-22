#ifndef UTILS_H
#define UTILS_H

#include <chrono>

#include "types.h"

namespace Robot
{
float clamp(float value, float min_value, float max_value);
float lowPassFilter(float previous_value, float sample, float alpha);
bool hasTimedOut(Timestamp last_update,
				 std::chrono::milliseconds timeout,
				 Timestamp now = SteadyClock::now());
bool isFresh(Timestamp last_update,
			 std::chrono::milliseconds timeout,
			 Timestamp now = SteadyClock::now());
}

#endif
