#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <condition_variable>
#include <mutex>
#include <string>

#include "climbing_fsm.h"
#include "motion_coordinator.h"
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
					SafetyManager& safety_manager);
	~RobotController();

	bool init();
	void requestStop(std::string message = "Stop requested.");
	void update();
	RobotState waitUntilFinished();
	void stopAll();
	void resetSystem();
	RobotState state() const;

private:
	void bindEventSources();

	ClimbingFsm& climbing_fsm_;
	MotionCoordinator& motion_coordinator_;
	StepDetector& step_detector_;
	SafetyManager& safety_manager_;

	mutable std::mutex mutex_;
	std::condition_variable state_cv_;
	bool callbacks_bound_{false};
	RobotState robot_state_;
};
}

#endif
