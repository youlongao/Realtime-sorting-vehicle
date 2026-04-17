#ifndef MOTION_COORDINATOR_H
#define MOTION_COORDINATOR_H

#include <functional>
#include <string>

#include "front_segment.h"
#include "middle_drive_module.h"
#include "middle_lift_module.h"
#include "rear_support_module.h"
#include "types.h"

namespace Robot
{
class MotionCoordinator
{
public:
	using StateEntryCallback = std::function<void(MotionState)>;

	MotionCoordinator(FrontSegment& front_segment,
					  MiddleLiftModule& middle_lift_module,
					  MiddleDriveModule& middle_drive_module,
					  RearSupportModule& rear_support_module,
					  IDriveSection* approach_assist_drive = nullptr,
					  StateEntryCallback state_entry_callback = {});

	bool executeState(MotionState current_state);
	bool isPhaseComplete(MotionState current_state) const;
	void stopAll();
	void resetPhases();

	enum class TimeoutKind { Sensor, Actuator };

	// Returns true if the coordinator detected a state timeout (either a
	// downward-sensor confirmation or an actuator-limit confirmation).
	// The caller is responsible for triggering a fault.
	bool hasStateTimeout() const;
	const std::string& stateTimeoutMessage() const;
	TimeoutKind stateTimeoutKind() const;

private:
	void enterState(MotionState next_state);
	void driveApproachAssist(float speed);
	void stopApproachAssist();
	void driveFrontAndMiddle(float speed);
	void stopFrontAndMiddle();
	// Returns true (and sets state_timed_out_) when the downward sensor has
	// not confirmed a landing within SENSOR_CONFIRM_TIMEOUT_S seconds.
	bool checkSensorTimeout(const char* state_name);
	// Returns true (and sets state_timed_out_) when a limit switch / actuator
	// has not reached its target within the given timeout (seconds).
	bool checkActuatorTimeout(const char* state_name, int timeout_s);

	FrontSegment& front_segment_;
	MiddleLiftModule& middle_lift_module_;
	MiddleDriveModule& middle_drive_module_;
	RearSupportModule& rear_support_module_;
	IDriveSection* approach_assist_drive_;
	StateEntryCallback state_entry_callback_;

	MotionState active_state_{MotionState::Idle};
	bool active_state_valid_{false};
	bool state_complete_{false};
	bool front_landing_extra_drive_active_{false};
	bool front_landing_extra_lift_active_{false};
	bool middle_landing_extra_drive_active_{false};

	Timestamp state_entry_time_{};
	Timestamp front_landing_extra_drive_start_time_{};
	Timestamp front_landing_extra_lift_start_time_{};
	Timestamp middle_landing_extra_drive_start_time_{};
	bool state_timed_out_{false};
	std::string state_timeout_message_;
	TimeoutKind state_timeout_kind_{TimeoutKind::Sensor};
};
}

#endif
