#ifndef OBSTACLE_AVOIDANCE
#define OBSTACLE_AVOIDANCE

#include <functional>

// define obstacle avoidance state callback
// true -> need to stop, false -> obstacle has been cleared
using ObstacleAlertCallback = std::function<void(bool)>;

class ObstacleAvoidance
{
public:
	// safe threshold -- cm
	ObstacleAvoidance(float safeThresholdCm);

	// register alert callback 
	void registerAlertCallback(ObstacleAlertCallback callback);

	// processing the latest distance from the ultrasonic sensor
	void processDistanceUpdate(float currentDistanceCm);

	// check the device is currently in obstacle avoidance lockout mode
	bool isObstacleDetected() const;

private:
	float threshold;	// safe distance
	bool currentlyBlocked;	// check it currently in a blocked state
	ObstacleAlertCallback m_alertCallback;
};

#endif