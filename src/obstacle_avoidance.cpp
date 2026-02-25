#include "obstacle_avoidance.h"
#include <iostream>

ObstacleAvoidance::ObstacleAvoidance(float safeThresholdCm)
	: threshold(safeThresholdCm), currentlyBlocked(false){ }

void ObstacleAvoidance::registerAlertCallback(ObstacleAlertCallback callback)
{
	m_alertCallback = std::move(callback);
}

bool ObstacleAvoidance::isObstacleDetected() const
{
	return currentlyBlocked;
}

void ObstacleAvoidance::processDistanceUpdate(float currentDistanceCm)
{
	if (currentDistanceCm < threshold) {
		// obstacles detected 
		if (!currentlyBlocked) {
			currentlyBlocked = true;
			std::cout << "[obstacle avoidance system] detects obstacles (" << currentDistanceCm << "cm ), Stop!\n";
			if (m_alertCallback) {
				m_alertCallback(true);	// notify the main system -> stop!
			}
		}
	}
	else {
		// no obstacles
		if (currentlyBlocked) {
			currentlyBlocked = false;
			std::cout << "[obstacle avoidance system] No obstacles! (" << currentDistanceCm << "cm ), Pass!\n";
			if (m_alertCallback) {
				m_alertCallback(false); // notify the main system -> release stop!
			}
		}
	}
}