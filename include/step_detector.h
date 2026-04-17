#ifndef STEP_DETECTOR_H
#define STEP_DETECTOR_H

#include <functional>
#include <mutex>

#include "config.h"
#include "front_distance_sensor.h"
#include "types.h"

namespace Robot
{
class StepDetector
{
public:
	/*
	front_distance_sensor: Ultrasonic sensor, determine the distance from the facade of the steps
	front_downward_sensor: The front downward-facing sensor is used to determine 
						   whether the area near the front wheels is suspended in the air or has landed
	middle_support_sensor: The mid-section downward-facing sensor is used to confirm 
						   whether the mid-section drive wheels have landed on the new support surface
	rear_support_sensor: The rear downward-facing sensor is used to confirm whether the rear support wheel has landed
	*/
	StepDetector(IFrontDistanceSensor& front_distance_sensor,
				 IDownwardSensor& front_downward_sensor,
				 IDownwardSensor* middle_support_sensor = nullptr,
				 IDownwardSensor* rear_support_sensor = nullptr);
	~StepDetector();

	void setUpdateCallback(std::function<void()> callback);
	StepAssessment detectStepEdge();	// Used to determine whether a step edge has been detected
	StepAssessment detectStepSurface();	// Used to determine whether a step support surface has been detected
	bool isReadyForClimb();		// This indicates whether the robot has met the conditions for "starting the initial climb"
	bool isStepCompleted();		// Is this step of the staircase complete
	StepAssessment latestAssessment() const;	// Read the latest comprehensive assessment results

private:
	StepAssessment buildAssessmentLocked() const;	// Under lock protection, the currently cached multi-channel 
													// sensor data is fused into a unified StepAssessment.
	void notifyUpdated();

	IFrontDistanceSensor& front_distance_sensor_;	// Forward ultrasonic interface
	IDownwardSensor& front_downward_sensor_;	// Front-end downward view interface
	IDownwardSensor* middle_support_sensor_;	// Mid-section support confirmation interface
	IDownwardSensor* rear_support_sensor_;	// Backend support confirmation interface
	std::function<void()> update_callback_;	// Callback used to notify the controller that new sensor data is available

	mutable std::mutex mutex_;	// Protect internal cached data and prevent multi-threaded access conflicts.
	DistanceReading last_distance_;	// Most recent forward distance reading
	DownwardReading last_front_downward_;	// The most recent forward downward reading
	DownwardReading last_middle_downward_;	// Recent mid-section support reading
	DownwardReading last_rear_downward_;	// Recent rear support reading
	StepAssessment latest_assessment_;	// The step evaluation results obtained from the most recent fusion calculation
};
}

#endif
