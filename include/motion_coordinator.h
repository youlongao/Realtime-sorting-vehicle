#ifndef MOTION_COORDINATOR_H
#define MOTION_COORDINATOR_H

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
	MotionCoordinator(FrontSegment& front_segment,
					  MiddleLiftModule& middle_lift_module,
					  MiddleDriveModule& middle_drive_module,
					  RearSupportModule& rear_support_module);

	bool executeFrontPhase(MotionState current_state);
	bool executeMiddleTransferPhase(MotionState current_state);
	bool executeRearTransferPhase(MotionState current_state);
	bool isPhaseComplete(MotionState current_state) const;
	void stopAll();
	void resetPhases();

private:
	FrontSegment& front_segment_;
	MiddleLiftModule& middle_lift_module_;
	MiddleDriveModule& middle_drive_module_;
	RearSupportModule& rear_support_module_;

	bool front_phase_complete_{false};
	bool middle_transfer_phase_complete_{false};
	bool rear_transfer_phase_complete_{false};
};
}

#endif
