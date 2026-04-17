#include "robot_controller.h"

#include <cmath>
#include <utility>

#include "logger.h"

namespace Robot
{
namespace
{
const char* MotionStateToString(const MotionState state)
{
	switch (state)
	{
	case MotionState::Idle:
		return "Idle";
	case MotionState::ApproachingStep:
		return "ApproachingStep";
	case MotionState::RearSliderBack:
		return "RearSliderBack";
	case MotionState::FrontLift:
		return "FrontLift";
	case MotionState::MiddleDriveToFrontLanding:
		return "MiddleDriveToFrontLanding";
	case MotionState::MiddleClimb:
		return "MiddleClimb";
	case MotionState::FrontDriveToMiddleLanding:
		return "FrontDriveToMiddleLanding";
	case MotionState::RearSliderForward:
		return "RearSliderForward";
	case MotionState::RearLift:
		return "RearLift";
	case MotionState::FinalDriveToRearLanding:
		return "FinalDriveToRearLanding";
	case MotionState::CycleReset:
		return "CycleReset";
	case MotionState::Completed:
		return "Completed";
	case MotionState::Fault:
	default:
		return "Fault";
	}
}
}

RobotController::RobotController(ClimbingFsm& climbing_fsm,
								 MotionCoordinator& motion_coordinator,
								 StepDetector& step_detector,
								 SafetyManager& safety_manager)
	: climbing_fsm_(climbing_fsm),
	  motion_coordinator_(motion_coordinator),
	  step_detector_(step_detector),
	  safety_manager_(safety_manager)
{
}

RobotController::~RobotController()
{
	step_detector_.setUpdateCallback({});
}

bool RobotController::init()
{
	motion_coordinator_.resetPhases();
	climbing_fsm_.transitionTo(MotionState::Idle);

	const auto safety_status = safety_manager_.checkAllSafetyConditions();
	{
		std::lock_guard<std::mutex> lock(mutex_);
		robot_state_.motion_state = climbing_fsm_.getCurrentState();
		robot_state_.step_assessment = step_detector_.latestAssessment();
		robot_state_.safety_status = safety_status;
		robot_state_.timestamp = SteadyClock::now();
	}

	if (safety_status.level == SafetyLevel::Fault)
	{
		stopAll();
		state_cv_.notify_all();
		return false;
	}

	bindEventSources();
	update();
	return true;
}

void RobotController::requestStop(std::string message)
{
	std::lock_guard<std::mutex> lock(mutex_);
	safety_manager_.emergencyStop(FaultCode::EmergencyStop, std::move(message));
	motion_coordinator_.stopAll();
	climbing_fsm_.transitionTo(MotionState::Fault);
	robot_state_.motion_state = MotionState::Fault;
	robot_state_.safety_status = safety_manager_.currentStatus();
	robot_state_.timestamp = SteadyClock::now();
	state_cv_.notify_all();
}

void RobotController::update()
{
	std::lock_guard<std::mutex> lock(mutex_);

	StepAssessment step_assessment;
	SafetyStatus safety_status;
	MotionState next_state = climbing_fsm_.getCurrentState();

	for (int transition_guard = 0; transition_guard < 12; ++transition_guard)
	{
		const auto current_state = climbing_fsm_.getCurrentState();
		safety_status = safety_manager_.checkAllSafetyConditions();
		step_assessment = step_detector_.detectStepEdge();

		if (current_state == MotionState::Completed || current_state == MotionState::Fault)
		{
			next_state = current_state;
			break;
		}

		if (safety_status.level == SafetyLevel::Fault)
		{
			next_state = climbing_fsm_.updateState(safety_status, false);
			const bool state_changed = climbing_fsm_.transitionTo(next_state);
			if (state_changed)
			{
				Logger::info(std::string("Controller motion state = ") + MotionStateToString(next_state));
			}
			break;
		}

		const bool state_complete = motion_coordinator_.executeState(current_state);

		if (motion_coordinator_.hasStateTimeout())
		{
			const std::string& timeout_msg = motion_coordinator_.stateTimeoutMessage();
			Logger::error(timeout_msg);
			const FaultCode fault_code =
				motion_coordinator_.stateTimeoutKind() == MotionCoordinator::TimeoutKind::Actuator
				? FaultCode::ActuatorFault
				: FaultCode::DownwardSensorFault;
			safety_manager_.emergencyStop(fault_code, timeout_msg);
			climbing_fsm_.handleError(fault_code);
			next_state = MotionState::Fault;
			safety_status = safety_manager_.currentStatus();
			break;
		}

		next_state = climbing_fsm_.updateState(safety_status, state_complete);
		const bool state_changed = climbing_fsm_.transitionTo(next_state);

		if (state_changed)
		{
			Logger::info(std::string("Controller motion state = ") + MotionStateToString(next_state));
		}

		if (!state_changed ||
			next_state == MotionState::Completed ||
			next_state == MotionState::Fault)
		{
			break;
		}
	}

	RobotState snapshot;
	snapshot.motion_state = climbing_fsm_.getCurrentState();
	snapshot.step_assessment = step_assessment;
	snapshot.safety_status = safety_status;
	snapshot.sensor_data.front_distance.distance_m = step_assessment.front_face_distance_m;
	snapshot.sensor_data.front_distance.valid = !std::isnan(step_assessment.front_face_distance_m);
	snapshot.sensor_data.front_distance.timestamp = step_assessment.timestamp;
	snapshot.timestamp = SteadyClock::now();
	robot_state_ = snapshot;

	if (next_state == MotionState::Completed || next_state == MotionState::Fault)
	{
		stopAll();
		state_cv_.notify_all();
	}
}

RobotState RobotController::waitUntilFinished()
{
	std::unique_lock<std::mutex> lock(mutex_);
	state_cv_.wait(lock, [this]() {
		return robot_state_.motion_state == MotionState::Completed ||
			   robot_state_.motion_state == MotionState::Fault;
	});

	return robot_state_;
}

void RobotController::stopAll()
{
	motion_coordinator_.stopAll();
}

void RobotController::resetSystem()
{
	safety_manager_.clearFault();
	motion_coordinator_.resetPhases();
	climbing_fsm_.transitionTo(MotionState::Idle);

	std::lock_guard<std::mutex> lock(mutex_);
	robot_state_ = {};
}

RobotState RobotController::state() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return robot_state_;
}

void RobotController::bindEventSources()
{
	if (callbacks_bound_)
	{
		return;
	}

	step_detector_.setUpdateCallback([this]() {
		update();
	});
	callbacks_bound_ = true;
}
}
