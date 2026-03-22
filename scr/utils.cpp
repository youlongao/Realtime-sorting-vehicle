#include "utils.h"

namespace Robot
{
float clamp(const float value, const float min_value, const float max_value)
{
	if (value < min_value)
	{
		return min_value;
	}

	if (value > max_value)
	{
		return max_value;
	}

	return value;
}

float lowPassFilter(const float previous_value, const float sample, const float alpha)
{
	return (alpha * previous_value) + ((1.0F - alpha) * sample);
}

bool hasTimedOut(const Timestamp last_update,
				 const std::chrono::milliseconds timeout,
				 const Timestamp now)
{
	return now - last_update > timeout;
}

bool isFresh(const Timestamp last_update,
			 const std::chrono::milliseconds timeout,
			 const Timestamp now)
{
	return !hasTimedOut(last_update, timeout, now);
}
}
