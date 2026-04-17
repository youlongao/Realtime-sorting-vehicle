#include "climbing_fsm.h"

#include "config.h"

namespace Robot
{
MotionState ClimbingFsm::updateState(const SafetyStatus& safety_status,
									 const bool phase_complete)
{
	if (safety_status.level == SafetyLevel::Fault)
	{
		current_state_ = MotionState::Fault;
		return current_state_;
	}

	const bool state_complete = phase_complete;

	switch (current_state_)
	{
	case MotionState::Idle:
		return MotionState::ApproachingStep;
	case MotionState::ApproachingStep:
		return state_complete ? MotionState::RearSliderBack : MotionState::ApproachingStep;
	case MotionState::RearSliderBack:
		return state_complete ? MotionState::FrontLift : MotionState::RearSliderBack;
	case MotionState::FrontLift:
		return state_complete ? MotionState::MiddleDriveToFrontLanding : MotionState::FrontLift;
	case MotionState::MiddleDriveToFrontLanding:
		return state_complete ? MotionState::MiddleClimb : MotionState::MiddleDriveToFrontLanding;
	case MotionState::MiddleClimb:
		return state_complete ? MotionState::FrontDriveToMiddleLanding : MotionState::MiddleClimb;
	case MotionState::FrontDriveToMiddleLanding:
		return state_complete ? MotionState::RearSliderForward : MotionState::FrontDriveToMiddleLanding;
	case MotionState::RearSliderForward:
		return state_complete ? MotionState::RearLift : MotionState::RearSliderForward;
	case MotionState::RearLift:
		return state_complete ? MotionState::FinalDriveToRearLanding : MotionState::RearLift;
	case MotionState::FinalDriveToRearLanding:
		if (!state_complete)
		{
			return MotionState::FinalDriveToRearLanding;
		}
		++climb_cycles_completed_;
		if (RobotConfig::Motion::MAX_CLIMB_CYCLES > 0 &&
			climb_cycles_completed_ >= RobotConfig::Motion::MAX_CLIMB_CYCLES)
		{
			return MotionState::Completed;
		}
		return MotionState::CycleReset;
	case MotionState::CycleReset:
		return state_complete ? MotionState::ApproachingStep : MotionState::CycleReset;
	case MotionState::Completed:
		return MotionState::Completed;
	case MotionState::Fault:
	default:
		return MotionState::Fault;
	}
}

bool ClimbingFsm::transitionTo(const MotionState next_state)
{
	if (current_state_ == next_state)
	{
		return false;
	}

	if (next_state == MotionState::Idle)
	{
		climb_cycles_completed_ = 0;
	}

	current_state_ = next_state;
	return true;
}

void ClimbingFsm::handleError(const FaultCode fault)
{
	last_fault_ = fault;
	current_state_ = MotionState::Fault;
}

MotionState ClimbingFsm::getCurrentState() const
{
	return current_state_;
}
}
