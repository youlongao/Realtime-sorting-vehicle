#ifndef TYPES_H
#define TYPES_H

#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>

namespace Robot
{
using SteadyClock = std::chrono::steady_clock;
using Timestamp = SteadyClock::time_point;

enum class FaultCode
{
	None,
	EmergencyStop,
	FrontDistanceTimeout,
	DownwardSensorFault,
	ImuFault,
	OverPitch,
	OverRoll,
	LimitSwitchTriggered,
	EncoderFault,
	ActuatorFault,
	LiftUpperLimit,
	LiftLowerLimit,
	SliderFault
};

enum class MotionState
{
	Idle,
	ApproachingStep,
	FrontClimb,
	MiddleTransfer,
	RearTransfer,
	Completed,
	Fault
};

enum class SafetyLevel
{
	Ok,
	Warning,
	Fault
};

enum class EdgeConfidence
{
	Unknown,
	Low,
	Medium,
	High
};

struct PoseData
{
	float pitch_deg{0.0F};
	float roll_deg{0.0F};
	float yaw_deg{0.0F};
	Timestamp timestamp{SteadyClock::now()};
	bool valid{false};
};

struct DistanceReading
{
	float distance_m{std::numeric_limits<float>::quiet_NaN()};
	Timestamp timestamp{SteadyClock::now()};
	bool valid{false};
};

struct DownwardReading
{
	bool on_step_surface{false};
	bool edge_detected{false};
	bool drop_detected{false};
	Timestamp timestamp{SteadyClock::now()};
	bool valid{false};
};

struct AxisState
{
	float position_m{0.0F};
	bool homed{false};
	bool in_motion{false};
	bool at_upper_limit{false};
	bool at_lower_limit{false};
	Timestamp timestamp{SteadyClock::now()};
};

struct EncoderSample
{
	std::int64_t ticks{0};
	float speed_mps{0.0F};
	float distance_m{0.0F};
	Timestamp timestamp{SteadyClock::now()};
	bool valid{false};
};

struct LimitSwitchState
{
	bool triggered{false};
	Timestamp timestamp{SteadyClock::now()};
	bool valid{false};
};

struct StepAssessment
{
	bool edge_detected{false};
	bool surface_detected{false};
	bool ready_for_climb{false};
	bool middle_surface_detected{false};
	bool rear_surface_detected{false};
	bool ready_for_middle_transfer{false};
	bool ready_for_rear_transfer{false};
	bool step_completed{false};
	float front_face_distance_m{std::numeric_limits<float>::quiet_NaN()};
	EdgeConfidence confidence{EdgeConfidence::Unknown};
	Timestamp timestamp{SteadyClock::now()};
};

struct SafetyStatus
{
	SafetyLevel level{SafetyLevel::Ok};
	FaultCode fault{FaultCode::None};
	std::string message{"safe"};
	Timestamp timestamp{SteadyClock::now()};
	bool latched{false};
};

struct SensorData
{
	PoseData pose{};
	DistanceReading front_distance{};
	DownwardReading downward{};
	EncoderSample encoder{};
	LimitSwitchState upper_limit{};
	LimitSwitchState lower_limit{};
};

struct RobotState
{
	MotionState motion_state{MotionState::Idle};
	SensorData sensor_data{};
	StepAssessment step_assessment{};
	SafetyStatus safety_status{};
	Timestamp timestamp{SteadyClock::now()};
};

using PoseCallback = std::function<void(const PoseData&)>;
using DistanceCallback = std::function<void(const DistanceReading&)>;
using DownwardCallback = std::function<void(const DownwardReading&)>;
using EncoderCallback = std::function<void(const EncoderSample&)>;
using LimitSwitchCallback = std::function<void(const LimitSwitchState&)>;
using SafetyCallback = std::function<void(const SafetyStatus&)>;
}

#endif
