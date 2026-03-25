#ifndef CLIMBING_FSM_H
#define CLIMBING_FSM_H

#include "types.h"

namespace Robot
{
class ClimbingFsm
{
public:
	MotionState updateState(const StepAssessment& step_assessment,
							const SafetyStatus& safety_status,
							bool front_phase_complete,
							bool middle_transfer_phase_complete,
							bool rear_transfer_phase_complete);
	bool transitionTo(MotionState next_state);
	void handleError(FaultCode fault);
	MotionState getCurrentState() const;

private:
	MotionState current_state_{MotionState::Idle};
	FaultCode last_fault_{FaultCode::None};
};
}

#endif
