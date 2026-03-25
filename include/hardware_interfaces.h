#ifndef HARDWARE_INTERFACES_H
#define HARDWARE_INTERFACES_H

#include <chrono>

#include "types.h"

namespace Robot
{
class IDriveSection
{
public:
	virtual ~IDriveSection() = default;
	virtual void setNormalizedSpeed(float left_speed, float right_speed) = 0;
	virtual void stop() = 0;
	virtual void brake() = 0;
};

class ILinearAxis
{
public:
	virtual ~ILinearAxis() = default;
	virtual void moveNormalized(float command) = 0;
	virtual void holdPosition() = 0;
	virtual void stop() = 0;
	virtual AxisState getAxisState() const = 0;
};

class IFrontDistanceSensor
{
public:
	virtual ~IFrontDistanceSensor() = default;
	virtual DistanceReading readBlocking(std::chrono::microseconds timeout) = 0;
	virtual DistanceReading latest() const = 0;
	virtual void setCallback(DistanceCallback callback) = 0;
};

class IDownwardSensor
{
public:
	virtual ~IDownwardSensor() = default;
	virtual DownwardReading latest() const = 0;
	virtual bool waitForEdge(std::chrono::milliseconds timeout) = 0;
	virtual void setCallback(DownwardCallback callback) = 0;
};

class IImuSensor
{
public:
	virtual ~IImuSensor() = default;
	virtual PoseData latestPose() const = 0;
	virtual void setCallback(PoseCallback callback) = 0;
};

class ILimitSwitch
{
public:
	virtual ~ILimitSwitch() = default;
	virtual LimitSwitchState latestState() const = 0;
	virtual bool waitForTrigger(std::chrono::milliseconds timeout) = 0;
	virtual void setCallback(LimitSwitchCallback callback) = 0;
};
}

#endif
