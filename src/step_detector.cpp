#include "step_detector.h"

#include <cmath>
#include <utility>

#include "utils.h"

namespace Robot
{
	// Register sensor callbacks to ensure the system always keeps the latest sensor readings
StepDetector::StepDetector(IFrontDistanceSensor& front_distance_sensor,
						   IDownwardSensor& front_downward_sensor,
						   IDownwardSensor* middle_support_sensor,
						   IDownwardSensor* rear_support_sensor)
	: front_distance_sensor_(front_distance_sensor),
	  front_downward_sensor_(front_downward_sensor),
	  middle_support_sensor_(middle_support_sensor),
	  rear_support_sensor_(rear_support_sensor)
{
	// Initialize the most recent sensor cache to prevent the internal state
	//  from being empty when the object is first created
	last_distance_ = front_distance_sensor_.latest();
	last_front_downward_ = front_downward_sensor_.latest();

	if (middle_support_sensor_ != nullptr)
	{
		last_middle_downward_ = middle_support_sensor_->latest();
	}
	if (rear_support_sensor_ != nullptr)
	{
		last_rear_downward_ = rear_support_sensor_->latest();
	}

	// Register forward distance sensor callback
	// Update the internal cache each time a new ultrasound reading is obtained
	front_distance_sensor_.setCallback([this](const DistanceReading& reading) {
		{
			std::lock_guard<std::mutex> lock(mutex_);
			last_distance_ = reading;
		}

		notifyUpdated();
	});

	// Downward sensor callback before registration
	// Update the front-ground/floating state cache each time the front-view state changes
	front_downward_sensor_.setCallback([this](const DownwardReading& reading) {
		{
			std::lock_guard<std::mutex> lock(mutex_);
			last_front_downward_ = reading;
		}

		notifyUpdated();
	});

	// If the mid-section support sensor exists, then register the mid-section state update callback
	if (middle_support_sensor_ != nullptr)
	{
		middle_support_sensor_->setCallback([this](const DownwardReading& reading) {
			{
				std::lock_guard<std::mutex> lock(mutex_);
				last_middle_downward_ = reading;
			}

			notifyUpdated();
		});
	}

	// If the downstream support sensor exists, register its downstream state update callback
	if (rear_support_sensor_ != nullptr)
	{
		rear_support_sensor_->setCallback([this](const DownwardReading& reading) {
			{
				std::lock_guard<std::mutex> lock(mutex_);
				last_rear_downward_ = reading;
			}

			notifyUpdated();
		});
	}
}

StepDetector::~StepDetector()
{
	front_distance_sensor_.setCallback({});
	front_downward_sensor_.setCallback({});
	if (middle_support_sensor_ != nullptr)
	{
		middle_support_sensor_->setCallback({});
	}
	if (rear_support_sensor_ != nullptr)
	{
		rear_support_sensor_->setCallback({});
	}
}

void StepDetector::setUpdateCallback(std::function<void()> callback)
{
	std::lock_guard<std::mutex> lock(mutex_);
	update_callback_ = std::move(callback);
}

// Detect the current step edge state
// Generate the latest comprehensive evaluation results
StepAssessment StepDetector::detectStepEdge()
{
	std::lock_guard<std::mutex> lock(mutex_);
	latest_assessment_ = buildAssessmentLocked();
	return latest_assessment_;
}

// Detect the current status of the step support surface
// It shares the same set of fusion judgment logic as detectStepEdge().
StepAssessment StepDetector::detectStepSurface()
{
	std::lock_guard<std::mutex> lock(mutex_);
	latest_assessment_ = buildAssessmentLocked();
	return latest_assessment_;
}

// Determine if the conditions for starting the climb from the previous stage have been met
bool StepDetector::isReadyForClimb()
{
	return detectStepEdge().ready_for_climb;
}

// Determine if the current round of the previous action has been completed
bool StepDetector::isStepCompleted()
{
	return detectStepSurface().step_completed;
}

// Obtain the latest comprehensive evaluation results
StepAssessment StepDetector::latestAssessment() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return latest_assessment_;
}

// Constructing a unified step evaluation result under locked conditions
StepAssessment StepDetector::buildAssessmentLocked() const
{
	StepAssessment assessment;

	// Record the current evaluation timestamp
	assessment.timestamp = SteadyClock::now();

	// Save the most recent forward distance for logging and status output
	assessment.front_face_distance_m = last_distance_.distance_m;

	// Determine if the forward distance data is valid and fresh
	const bool front_fresh = last_distance_.valid &&
		isFresh(last_distance_.timestamp, std::chrono::milliseconds(RobotConfig::Sensors::SENSOR_STALE_MS));
	
	// Determine if the forward downward look data is valid and fresh.
	// These are event-driven digital sensors (MCP23017 interrupt path): a
	// stable pin produces no new event, so use the wider DOWNWARD_SENSOR_STALE_MS
	// window rather than the ultrasonic SENSOR_STALE_MS window.
		const bool front_downward_fresh = last_front_downward_.valid &&
		isFresh(last_front_downward_.timestamp, std::chrono::milliseconds(RobotConfig::Sensors::DOWNWARD_SENSOR_STALE_MS));

	// Determine if the mid-range support data is valid and recent
		const bool middle_downward_fresh = middle_support_sensor_ != nullptr &&
		last_middle_downward_.valid &&
		isFresh(last_middle_downward_.timestamp, std::chrono::milliseconds(RobotConfig::Sensors::DOWNWARD_SENSOR_STALE_MS));

	// Determine if the subsequent support data is valid and recent
		const bool rear_downward_fresh = rear_support_sensor_ != nullptr &&
		last_rear_downward_.valid &&
		isFresh(last_rear_downward_.timestamp, std::chrono::milliseconds(RobotConfig::Sensors::DOWNWARD_SENSOR_STALE_MS));
	
	// Determine if the forward distance is within the "step facade detection range"
	const bool front_distance_in_band =
		front_fresh &&
		last_distance_.distance_m >= RobotConfig::Sensors::STEP_FACE_MIN_DISTANCE_M &&
		last_distance_.distance_m <= RobotConfig::Sensors::STEP_FACE_MAX_DISTANCE_M;

	// Determine if step edge is detected
	// It needs to simultaneously meet the conditions of "a step facade detected ahead" 
	// and "the area under the front wheels is now suspended in the air"
	assessment.edge_detected = front_distance_in_band &&
							   front_downward_fresh && last_front_downward_.drop_detected;

	// Determine if the front part has landed on the step surface
	assessment.surface_detected = front_downward_fresh && last_front_downward_.on_step_surface;
	
	// Determine if the middle section has landed on the new support surface
	assessment.middle_surface_detected = middle_downward_fresh && last_middle_downward_.on_step_surface;

	// Determine whether the latter part has landed on the new support surface
	assessment.rear_surface_detected = rear_downward_fresh && last_rear_downward_.on_step_surface;
	
	// Determine if you are ready to begin the initial climb
	// It requires posture safety, sufficiently close forward distance, effective forward downward look, and edge detection
	assessment.ready_for_climb = front_fresh &&
								 front_downward_fresh &&
								 last_distance_.distance_m <= RobotConfig::Sensors::READY_TO_CLIMB_DISTANCE_M &&
								 assessment.edge_detected;
	
	// Once the initial phase has landed, the mid-course transfer phase can begin
	assessment.ready_for_middle_transfer = assessment.surface_detected;
	
	// Once the mid-section has landed, the subsequent transfer phase can begin
	assessment.ready_for_rear_transfer = assessment.middle_surface_detected;
	
	// When the rear support wheel has landed, the upper stage of this cycle is considered complete
	assessment.step_completed = assessment.rear_surface_detected;

	// Based on the current assessment results, assign a confidence level
	if (assessment.ready_for_climb)
	{
		assessment.confidence = EdgeConfidence::High;
	}
	else if (assessment.edge_detected)
	{
		assessment.confidence = EdgeConfidence::Medium;
	}
	else if (front_fresh || front_downward_fresh || middle_downward_fresh || rear_downward_fresh)
	{
		assessment.confidence = EdgeConfidence::Low;
	}

	return assessment;
}

void StepDetector::notifyUpdated()
{
	std::function<void()> callback;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		callback = update_callback_;
	}

	if (callback)
	{
		callback();
	}
}
}
