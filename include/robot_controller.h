#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <mutex>

#include "climbing_fsm.h"
#include "motion_coordinator.h"
#include "pose_monitor.h"
#include "safety_manager.h"
#include "step_detector.h"
#include "types.h"

namespace Robot
{
class RobotController
{
public:
	RobotController(ClimbingFsm& climbing_fsm,
					MotionCoordinator& motion_coordinator,
					StepDetector& step_detector,
					PoseMonitor& pose_monitor,
					SafetyManager& safety_manager);

	bool init();
	void update();
	void stopAll();
	void resetSystem();
	RobotState state() const;

private:
	ClimbingFsm& climbing_fsm_;
	MotionCoordinator& motion_coordinator_;
	StepDetector& step_detector_;
	PoseMonitor& pose_monitor_;
	SafetyManager& safety_manager_;

	mutable std::mutex mutex_;
	RobotState robot_state_;
};
}

#endif
