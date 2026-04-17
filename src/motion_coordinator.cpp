#include "motion_coordinator.h"

#include <chrono>
#include <string>
#include <utility>

#include "config.h"
#include "logger.h"

namespace Robot
{
MotionCoordinator::MotionCoordinator(FrontSegment& front_segment,
									 MiddleLiftModule& middle_lift_module,
									 MiddleDriveModule& middle_drive_module,
									 RearSupportModule& rear_support_module,
									 IDriveSection* approach_assist_drive,
									 StateEntryCallback state_entry_callback)
	: front_segment_(front_segment),
	  middle_lift_module_(middle_lift_module),
	  middle_drive_module_(middle_drive_module),
	  rear_support_module_(rear_support_module),
	  approach_assist_drive_(approach_assist_drive),
	  state_entry_callback_(std::move(state_entry_callback))
{
}

bool MotionCoordinator::executeState(const MotionState current_state)
{
	if (!active_state_valid_ || active_state_ != current_state)
	{
		enterState(current_state);
	}

	state_complete_ = false;

	switch (current_state)
	{
	case MotionState::Idle:
		state_complete_ = true;
		break;

	case MotionState::ApproachingStep:
		if (checkActuatorTimeout("ApproachingStep", RobotConfig::Motion::APPROACH_TIMEOUT_S))
		{
			front_segment_.stopDrive();
			stopApproachAssist();
		}
		else
		{
			state_complete_ = front_segment_.approachStep();
			if (state_complete_)
			{
				stopApproachAssist();
			}
			else
			{
				driveApproachAssist(front_segment_.approachAssistSpeed());
			}
		}
		break;

	case MotionState::RearSliderBack:
		front_segment_.brakeDrive();
		middle_drive_module_.holdPosition();
		if (!checkActuatorTimeout("RearSliderBack", RobotConfig::Motion::SLIDER_CONFIRM_TIMEOUT_S))
		{
			state_complete_ = rear_support_module_.moveSlideBackwardUntilLimit();
		}
		break;

	case MotionState::FrontLift:
		middle_drive_module_.holdPosition();
		if (!checkActuatorTimeout("FrontLift", RobotConfig::Motion::LIFT_CONFIRM_TIMEOUT_S))
		{
			state_complete_ = front_segment_.liftFrontUntilClearance();
		}
		break;

	case MotionState::MiddleDriveToFrontLanding:
		if (front_landing_extra_drive_active_)
		{
			const auto extra_drive_elapsed = SteadyClock::now() - front_landing_extra_drive_start_time_;
			if (extra_drive_elapsed >=
				std::chrono::milliseconds(RobotConfig::Motion::FRONT_LANDING_EXTRA_DRIVE_MS))
			{
				stopFrontAndMiddle();
				front_landing_extra_drive_active_ = false;
				front_landing_extra_lift_active_ = true;
				front_landing_extra_lift_start_time_ = SteadyClock::now();
				Logger::info(
					"Front landing extra drive complete; continuing front lift for " +
					std::to_string(RobotConfig::Motion::FRONT_LANDING_EXTRA_LIFT_S) +
					"s before middle climb.");
				front_segment_.continueFrontLift();
			}
			else
			{
				driveFrontAndMiddle(RobotConfig::Motion::CREEP_SPEED);
			}
		}
		else if (front_landing_extra_lift_active_)
		{
			front_segment_.brakeDrive();
			stopApproachAssist();
			const auto extra_lift_elapsed = SteadyClock::now() - front_landing_extra_lift_start_time_;
			if (extra_lift_elapsed >= std::chrono::seconds(RobotConfig::Motion::FRONT_LANDING_EXTRA_LIFT_S))
			{
				front_segment_.holdFrontLift();
				state_complete_ = true;
			}
			else
			{
				front_segment_.continueFrontLift();
			}
		}
		else if (front_segment_.isSurfaceConfirmed())
		{
			front_landing_extra_drive_active_ = true;
			front_landing_extra_drive_start_time_ = SteadyClock::now();
			Logger::info(
				"Front landing confirmed; driving front and middle wheels forward for " +
				std::to_string(RobotConfig::Motion::FRONT_LANDING_EXTRA_DRIVE_MS) +
				"ms.");
			driveFrontAndMiddle(RobotConfig::Motion::CREEP_SPEED);
		}
		else if (checkSensorTimeout("MiddleDriveToFrontLanding"))
		{
			stopFrontAndMiddle();
			front_segment_.holdFrontLift();
		}
		else
		{
			driveFrontAndMiddle(RobotConfig::Motion::CREEP_SPEED);
		}
		break;

	case MotionState::MiddleClimb:
		front_segment_.brakeDrive();
		middle_drive_module_.holdPosition();
		if (!checkActuatorTimeout("MiddleClimb", RobotConfig::Motion::LIFT_CONFIRM_TIMEOUT_S))
		{
			const bool front_lift_lowered = middle_lift_module_.lowerUntilLowerLimit();
			(void)rear_support_module_.lowerRearUntilLowerLimit();
			if (front_lift_lowered)
			{
				rear_support_module_.stabilizeSupport();
				state_complete_ = true;
			}
		}
		break;

	case MotionState::FrontDriveToMiddleLanding:
		if (middle_landing_extra_drive_active_)
		{
			const auto extra_drive_elapsed = SteadyClock::now() - middle_landing_extra_drive_start_time_;
			if (extra_drive_elapsed >=
				std::chrono::milliseconds(RobotConfig::Motion::MIDDLE_LANDING_EXTRA_DRIVE_MS))
			{
				stopFrontAndMiddle();
				middle_landing_extra_drive_active_ = false;
				state_complete_ = true;
			}
			else
			{
				driveFrontAndMiddle(RobotConfig::Motion::CREEP_SPEED);
			}
		}
		else if (checkSensorTimeout("FrontDriveToMiddleLanding"))
		{
			stopFrontAndMiddle();
		}
		else if (front_segment_.isSurfaceConfirmed() && middle_drive_module_.isSupportConfirmed())
		{
			middle_landing_extra_drive_active_ = true;
			middle_landing_extra_drive_start_time_ = SteadyClock::now();
			Logger::info(
				"Front and middle landing confirmed; driving front and middle wheels forward for " +
				std::to_string(RobotConfig::Motion::MIDDLE_LANDING_EXTRA_DRIVE_MS) +
				"ms.");
			driveFrontAndMiddle(RobotConfig::Motion::CREEP_SPEED);
		}
		else
		{
			front_segment_.driveForward(RobotConfig::Motion::CREEP_SPEED);
			stopApproachAssist();
		}
		break;

	case MotionState::RearSliderForward:
		front_segment_.brakeDrive();
		middle_drive_module_.holdPosition();
		if (!checkActuatorTimeout("RearSliderForward", RobotConfig::Motion::SLIDER_CONFIRM_TIMEOUT_S))
		{
			state_complete_ = rear_support_module_.moveSlideForwardUntilLimit();
		}
		break;

	case MotionState::RearLift:
		front_segment_.brakeDrive();
		middle_drive_module_.holdPosition();
		if (!checkActuatorTimeout("RearLift", RobotConfig::Motion::LIFT_CONFIRM_TIMEOUT_S))
		{
			state_complete_ = rear_support_module_.liftRearUntilUpperLimit();
		}
		break;

	case MotionState::FinalDriveToRearLanding:
		if (checkSensorTimeout("FinalDriveToRearLanding"))
		{
			front_segment_.stopDrive();
			stopApproachAssist();
		}
		else if (rear_support_module_.isSupportConfirmed())
		{
			front_segment_.stopDrive();
			stopApproachAssist();
			rear_support_module_.stabilizeSupport();
			state_complete_ = true;
		}
		else
		{
			front_segment_.driveForward(RobotConfig::Motion::CREEP_SPEED);
			driveApproachAssist(RobotConfig::Motion::CREEP_SPEED);
		}
		break;

	case MotionState::CycleReset:
		front_segment_.brakeDrive();
		middle_drive_module_.holdPosition();
		if (!checkActuatorTimeout("CycleReset", RobotConfig::Motion::SLIDER_CONFIRM_TIMEOUT_S))
		{
			state_complete_ = rear_support_module_.moveSlideBackwardUntilLimit();
		}
		break;

	case MotionState::Completed:
	case MotionState::Fault:
	default:
		stopAll();
		state_complete_ = true;
		break;
	}

	return state_complete_;
}

bool MotionCoordinator::isPhaseComplete(const MotionState) const
{
	return state_complete_;
}

void MotionCoordinator::stopAll()
{
	front_segment_.stopFrontSegment();
	middle_lift_module_.holdPosition();
	middle_drive_module_.holdPosition();
	rear_support_module_.stabilizeSupport();
	resetPhases();
}

void MotionCoordinator::resetPhases()
{
	state_complete_ = false;
	active_state_valid_ = false;
	front_landing_extra_drive_active_ = false;
	front_landing_extra_lift_active_ = false;
	middle_landing_extra_drive_active_ = false;
	front_landing_extra_drive_start_time_ = Timestamp{};
	front_landing_extra_lift_start_time_ = Timestamp{};
	middle_landing_extra_drive_start_time_ = Timestamp{};
	state_timed_out_ = false;
	state_timeout_message_.clear();
	state_timeout_kind_ = TimeoutKind::Sensor;
}

void MotionCoordinator::enterState(const MotionState next_state)
{
	active_state_ = next_state;
	active_state_valid_ = true;
	state_complete_ = false;
	state_entry_time_ = SteadyClock::now();
	front_landing_extra_drive_active_ = false;
	front_landing_extra_lift_active_ = false;
	middle_landing_extra_drive_active_ = false;
	front_landing_extra_drive_start_time_ = Timestamp{};
	front_landing_extra_lift_start_time_ = Timestamp{};
	middle_landing_extra_drive_start_time_ = Timestamp{};
	state_timed_out_ = false;
	state_timeout_message_.clear();
	state_timeout_kind_ = TimeoutKind::Sensor;

	if (state_entry_callback_)
	{
		state_entry_callback_(next_state);
	}
}

bool MotionCoordinator::hasStateTimeout() const
{
	return state_timed_out_;
}

const std::string& MotionCoordinator::stateTimeoutMessage() const
{
	return state_timeout_message_;
}

MotionCoordinator::TimeoutKind MotionCoordinator::stateTimeoutKind() const
{
	return state_timeout_kind_;
}

bool MotionCoordinator::checkSensorTimeout(const char* state_name)
{
	if (state_timed_out_)
	{
		return true;
	}

	const auto elapsed = SteadyClock::now() - state_entry_time_;
	if (elapsed >= std::chrono::seconds(RobotConfig::Motion::SENSOR_CONFIRM_TIMEOUT_S))
	{
		state_timed_out_ = true;
		state_timeout_kind_ = TimeoutKind::Sensor;
		state_timeout_message_ =
			std::string("Sensor confirmation timeout after ") +
			std::to_string(RobotConfig::Motion::SENSOR_CONFIRM_TIMEOUT_S) +
			"s in state: " + state_name;
		return true;
	}

	return false;
}

bool MotionCoordinator::checkActuatorTimeout(const char* state_name, const int timeout_s)
{
	if (state_timed_out_)
	{
		return true;
	}

	const auto elapsed = SteadyClock::now() - state_entry_time_;
	if (elapsed >= std::chrono::seconds(timeout_s))
	{
		state_timed_out_ = true;
		state_timeout_kind_ = TimeoutKind::Actuator;
		state_timeout_message_ =
			std::string("Actuator confirmation timeout after ") +
			std::to_string(timeout_s) +
			"s in state: " + state_name;
		return true;
	}

	return false;
}

void MotionCoordinator::driveApproachAssist(const float speed)
{
	if (approach_assist_drive_ != nullptr)
	{
		approach_assist_drive_->setNormalizedSpeed(speed, speed);
	}
}

void MotionCoordinator::driveFrontAndMiddle(const float speed)
{
	front_segment_.driveForward(speed);
	driveApproachAssist(speed);
}

void MotionCoordinator::stopFrontAndMiddle()
{
	front_segment_.stopDrive();
	stopApproachAssist();
}

void MotionCoordinator::stopApproachAssist()
{
	if (approach_assist_drive_ != nullptr)
	{
		approach_assist_drive_->stop();
	}
}
}
