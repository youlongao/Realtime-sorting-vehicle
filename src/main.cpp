#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "climbing_fsm.h"
#include "front_distance_sensor.h"
#include "front_segment.h"
#include "imu_sensor.h"
#include "linear_actuator.h"
#include "logger.h"
#include "mcp23017_downward_sensor.h"
#include "mcp23017_driver.h"
#include "mcp23017_limit_switch.h"
#include "middle_drive_module.h"
#include "middle_lift_module.h"
#include "motion_coordinator.h"
#include "motor_driver.h"
#include "pose_monitor.h"
#include "rear_support_module.h"
#include "robot_controller.h"
#include "safety_manager.h"
#include "front_balance_slider.h"
#include "step_detector.h"

using namespace Robot;

namespace
{
constexpr float kInitialFrontSliderExtensionM = RobotConfig::Geometry::SLIDER_HOME_OFFSET_M;
constexpr auto kSliderHomeTimeout = std::chrono::seconds(2);

class FixedPoseImuSensor final : public IImuSensor
{
public:
	FixedPoseImuSensor()
	{
		pose_.pitch_deg = 0.0F;
		pose_.roll_deg = 0.0F;
		pose_.yaw_deg = 0.0F;
		pose_.timestamp = SteadyClock::now();
		pose_.valid = true;
	}

	PoseData latestPose() const override
	{
		return pose_;
	}

	void setCallback(PoseCallback callback) override
	{
		callback_ = std::move(callback);
		if (callback_)
		{
			callback_(pose_);
		}
	}

private:
	PoseData pose_{};
	PoseCallback callback_;
};

bool WaitForValidImuSample(ImuSensor& imu_sensor, const std::chrono::milliseconds timeout)
{
	if (imu_sensor.latestPose().valid)
	{
		return true;
	}

	std::mutex mutex;
	std::condition_variable cv;
	bool pose_ready = false;

	imu_sensor.setCallback([&mutex, &cv, &pose_ready](const PoseData& pose) {
		if (!pose.valid)
		{
			return;
		}

		{
			std::lock_guard<std::mutex> lock(mutex);
			pose_ready = true;
		}

		cv.notify_all();
	});

	std::unique_lock<std::mutex> lock(mutex);
	(void)cv.wait_for(lock, timeout, [&pose_ready]() {
		return pose_ready;
	});
	return pose_ready || imu_sensor.latestPose().valid;
}

bool WaitForValidFrontDistanceSample(FrontDistanceSensor& front_sensor, const std::chrono::milliseconds timeout)
{
	if (front_sensor.latest().valid)
	{
		return true;
	}

	std::mutex mutex;
	std::condition_variable cv;
	bool sample_ready = false;

	front_sensor.setCallback([&mutex, &cv, &sample_ready](const DistanceReading& reading) {
		if (!reading.valid)
		{
			return;
		}

		{
			std::lock_guard<std::mutex> lock(mutex);
			sample_ready = true;
		}

		cv.notify_all();
	});

	std::unique_lock<std::mutex> lock(mutex);
	(void)cv.wait_for(lock, timeout, [&sample_ready]() {
		return sample_ready;
	});
	return sample_ready || front_sensor.latest().valid;
}

bool HomeFrontSliderIfNeeded(FrontBalanceSlider& front_slider,
							 ILinearAxis& front_slider_axis,
							 ILimitSwitch& front_slider_home_switch)
{
	if (front_slider_axis.getAxisState().homed)
	{
		return true;
	}

	front_slider.home();
	if (front_slider_axis.getAxisState().homed)
	{
		return true;
	}

	if (!front_slider_home_switch.waitForTrigger(kSliderHomeTimeout))
	{
		front_slider_axis.stop();
		Logger::error(
			"Front slider homing timed out. Start the robot with the front slider retracted.");
		return false;
	}

	front_slider.home();
	if (front_slider_axis.getAxisState().homed)
	{
		return true;
	}

	Logger::error(
		"Front slider failed to home from lower limit only. Start the robot with the front slider retracted.");
	return false;
}

int RunRobotController(IDriveSection& front_drive,
					   IDriveSection& middle_drive,
					   ILinearAxis& first_lift_axis,
					   ILinearAxis& front_slider_axis,
					   ILinearAxis& rear_slide_axis,
					   ILinearAxis& rear_lift_axis,
					   IFrontDistanceSensor& front_sensor,
					   IDownwardSensor& front_downward_sensor,
					   IDownwardSensor* middle_support_sensor,
					   IDownwardSensor* rear_support_sensor,
					   IImuSensor& imu_sensor,
					   ILimitSwitch& front_slider_home_switch)
{
	FrontBalanceSlider front_slider(front_slider_axis);
	if (!HomeFrontSliderIfNeeded(front_slider, front_slider_axis, front_slider_home_switch))
	{
		return 1;
	}
	front_slider.updateExtension(kInitialFrontSliderExtensionM);

	PoseMonitor pose_monitor(&imu_sensor);
	const auto initial_pose = imu_sensor.latestPose();
	if (initial_pose.valid)
	{
		pose_monitor.updatePose(initial_pose);
	}

	const auto middle_support_confirmed = [middle_support_sensor]() {
		return middle_support_sensor != nullptr && middle_support_sensor->latest().on_step_surface;
	};
	const auto rear_support_confirmed = [rear_support_sensor]() {
		return rear_support_sensor != nullptr && rear_support_sensor->latest().on_step_surface;
	};

	FrontSegment front_segment(
		front_drive,
		front_sensor,
		&first_lift_axis,
		[&front_downward_sensor]() { return front_downward_sensor.latest().on_step_surface; });
	MiddleLiftModule middle_lift(first_lift_axis, middle_support_confirmed);
	MiddleDriveModule middle_drive_module(middle_drive, middle_support_confirmed);
	RearSupportModule rear_support_module(&rear_slide_axis, &rear_lift_axis, rear_support_confirmed);
	StepDetector step_detector(
		front_sensor,
		front_downward_sensor,
		middle_support_sensor,
		rear_support_sensor,
		[&pose_monitor]() { return pose_monitor.isSafe(); });
	SafetyManager safety_manager(&pose_monitor);
	ClimbingFsm climbing_fsm;
	MotionCoordinator motion_coordinator(front_segment, middle_lift, middle_drive_module, rear_support_module);
	RobotController robot_controller(
		climbing_fsm,
		motion_coordinator,
		step_detector,
		pose_monitor,
		safety_manager);

	safety_manager.addRule([&front_sensor]() -> std::optional<SafetyStatus> {
		if (!front_sensor.latest().valid)
		{
			SafetyStatus status;
			status.level = SafetyLevel::Fault;
			status.fault = FaultCode::FrontDistanceTimeout;
			status.message = "Front distance sensor became invalid.";
			status.timestamp = SteadyClock::now();
			return status;
		}

		return std::nullopt;
	});

	safety_manager.addEmergencyStopHandler(
		[&front_segment, &middle_lift, &middle_drive_module, &rear_support_module]() {
			front_segment.stopFrontSegment();
			middle_lift.holdPosition();
			middle_drive_module.holdPosition();
			rear_support_module.stabilizeSupport();
		});

	if (!robot_controller.init())
	{
		Logger::error("Robot controller failed to initialise due to a safety fault.");
		return 1;
	}

	const auto robot_state = robot_controller.waitUntilFinished();
	if (robot_state.motion_state == MotionState::Completed)
	{
		Logger::info("Climbing cycle completed successfully.");
		return 0;
	}

	Logger::error("Robot controller entered fault state: " + robot_state.safety_status.message);
	return 1;
}

int RunHardwareBringup()
{
	Logger::info("Raspberry Pi Linux hardware mode starting.");

	auto mcp23017 = std::make_shared<Mcp23017Driver>();
	const auto warnIfStartFailed = [](const std::string& device_name,
									  const bool started,
									  const std::string& hint) {
		if (!started)
		{
			Logger::warn(device_name + " start failed. " + hint);
		}
	};

	warnIfStartFailed(
		"MCP23017 expander",
		mcp23017->start(),
		"Check SDA/SCL/VCC/GND wiring, RESET tied high, and address 0x20.");

	MotorDriver front_drive(
		"front",
		RobotConfig::MotorGPIO::FRONT_L_IN1,
		RobotConfig::MotorGPIO::FRONT_L_IN2,
		RobotConfig::MotorGPIO::FRONT_R_IN3,
		RobotConfig::MotorGPIO::FRONT_R_IN4);

	MotorDriver middle_drive(
		"middle",
		RobotConfig::MotorGPIO::MIDDLE_L_IN1,
		RobotConfig::MotorGPIO::MIDDLE_L_IN2,
		RobotConfig::MotorGPIO::MIDDLE_R_IN3,
		RobotConfig::MotorGPIO::MIDDLE_R_IN4);

	warnIfStartFailed(
		"Front drive",
		front_drive.start(),
		"Check front DRV8833 GPIO wiring: BCM17/18/27/22 -> IN1/IN2/IN3/IN4, and ensure EEP/nSLEEP is high.");
	warnIfStartFailed(
		"Middle drive",
		middle_drive.start(),
		"Check middle DRV8833 GPIO wiring: BCM23/24/25/8 -> IN1/IN2/IN3/IN4, and ensure EEP/nSLEEP is high.");

	FrontDistanceSensor front_sensor;
	const bool front_sensor_started = front_sensor.start();
	warnIfStartFailed("Front ultrasonic sensor", front_sensor_started, "Check TRIG/ECHO wiring.");
	if (front_sensor_started &&
		!WaitForValidFrontDistanceSample(
			front_sensor,
			std::chrono::milliseconds(RobotConfig::Sensors::SENSOR_STALE_MS)))
	{
		Logger::warn("Front ultrasonic sensor did not produce a valid sample during startup.");
	}

	Mcp23017DownwardSensor front_downward_sensor(
		mcp23017,
		RobotConfig::MCP23017::FRONT_DOWNWARD_DO,
		RobotConfig::Sensors::DOWNWARD_ACTIVE_ON_SURFACE);
	Mcp23017DownwardSensor middle_support_sensor(
		mcp23017,
		RobotConfig::MCP23017::MIDDLE_SUPPORT_DO,
		RobotConfig::Sensors::DOWNWARD_ACTIVE_ON_SURFACE);
	Mcp23017DownwardSensor rear_support_sensor(
		mcp23017,
		RobotConfig::MCP23017::REAR_SUPPORT_DO,
		RobotConfig::Sensors::DOWNWARD_ACTIVE_ON_SURFACE);
	warnIfStartFailed("Front downward edge sensor", front_downward_sensor.start(), "Check MCP23017 GPA0 wiring and TCRT5000 DO output.");
	warnIfStartFailed("Middle support sensor", middle_support_sensor.start(), "Check MCP23017 GPA1 wiring and TCRT5000 DO output.");
	warnIfStartFailed("Rear support sensor", rear_support_sensor.start(), "Check MCP23017 GPA2 wiring and TCRT5000 DO output.");

	Mcp23017LimitSwitch lift1_upper_limit(
		mcp23017,
		RobotConfig::MCP23017::LIFT_1_UPPER_LIMIT,
		LimitRole::Upper,
		RobotConfig::Limits::ACTIVE_LOW);
	Mcp23017LimitSwitch lift1_lower_limit(
		mcp23017,
		RobotConfig::MCP23017::LIFT_1_LOWER_LIMIT,
		LimitRole::Lower,
		RobotConfig::Limits::ACTIVE_LOW);
	Mcp23017LimitSwitch slide1_upper_limit(
		mcp23017,
		RobotConfig::MCP23017::SLIDE_1_UPPER_LIMIT,
		LimitRole::Upper,
		RobotConfig::Limits::ACTIVE_LOW);
	Mcp23017LimitSwitch slide1_lower_limit(
		mcp23017,
		RobotConfig::MCP23017::SLIDE_1_LOWER_LIMIT,
		LimitRole::Lower,
		RobotConfig::Limits::ACTIVE_LOW);
	Mcp23017LimitSwitch slide2_upper_limit(
		mcp23017,
		RobotConfig::MCP23017::SLIDE_2_UPPER_LIMIT,
		LimitRole::Upper,
		RobotConfig::Limits::ACTIVE_LOW);
	Mcp23017LimitSwitch slide2_lower_limit(
		mcp23017,
		RobotConfig::MCP23017::SLIDE_2_LOWER_LIMIT,
		LimitRole::Lower,
		RobotConfig::Limits::ACTIVE_LOW);
	Mcp23017LimitSwitch lift2_upper_limit(
		mcp23017,
		RobotConfig::MCP23017::LIFT_2_UPPER_LIMIT,
		LimitRole::Upper,
		RobotConfig::Limits::ACTIVE_LOW);
	Mcp23017LimitSwitch lift2_lower_limit(
		mcp23017,
		RobotConfig::MCP23017::LIFT_2_LOWER_LIMIT,
		LimitRole::Lower,
		RobotConfig::Limits::ACTIVE_LOW);

	warnIfStartFailed("Lift-1 upper limit", lift1_upper_limit.start(), "Check MCP23017 GPA3 and active-low limit wiring.");
	warnIfStartFailed("Lift-1 lower limit", lift1_lower_limit.start(), "Check MCP23017 GPA4 and active-low limit wiring.");
	warnIfStartFailed("Slide-1 upper limit", slide1_upper_limit.start(), "Check MCP23017 GPA5 and active-low limit wiring.");
	warnIfStartFailed("Slide-1 lower limit", slide1_lower_limit.start(), "Check MCP23017 GPA6 and active-low limit wiring.");
	warnIfStartFailed("Slide-2 upper limit", slide2_upper_limit.start(), "Check MCP23017 GPA7 and active-low limit wiring.");
	warnIfStartFailed("Slide-2 lower limit", slide2_lower_limit.start(), "Check MCP23017 GPB0 and active-low limit wiring.");
	warnIfStartFailed("Lift-2 upper limit", lift2_upper_limit.start(), "Check MCP23017 GPB1 and active-low limit wiring.");
	warnIfStartFailed("Lift-2 lower limit", lift2_lower_limit.start(), "Check MCP23017 GPB2 and active-low limit wiring.");

	LinearActuator first_lift_axis(
		RobotConfig::MotorGPIO::LIFT_1_IN1_IN3,
		RobotConfig::MotorGPIO::LIFT_1_IN2_IN4,
		&lift1_upper_limit,
		&lift1_lower_limit,
		RobotConfig::Geometry::BODY_LIFT_MAX_TRAVEL_M);
	LinearActuator front_slider_axis(
		RobotConfig::MotorGPIO::SLIDE_1_IN1_IN3,
		RobotConfig::MotorGPIO::SLIDE_1_IN2_IN4,
		&slide1_upper_limit,
		&slide1_lower_limit,
		RobotConfig::Geometry::SLIDER_MAX_TRAVEL_M);
	LinearActuator rear_slide_axis(
		RobotConfig::MotorGPIO::SLIDE_2_IN1_IN3,
		RobotConfig::MotorGPIO::SLIDE_2_IN2_IN4,
		&slide2_upper_limit,
		&slide2_lower_limit,
		RobotConfig::Geometry::SLIDER_MAX_TRAVEL_M);
	LinearActuator rear_lift_axis(
		RobotConfig::MotorGPIO::LIFT_2_IN1_IN3,
		RobotConfig::MotorGPIO::LIFT_2_IN2_IN4,
		&lift2_upper_limit,
		&lift2_lower_limit,
		RobotConfig::Geometry::BODY_LIFT_MAX_TRAVEL_M);

	warnIfStartFailed(
		"Lift-1 actuator",
		first_lift_axis.start(),
		"Check DRV8833 #1 direct GPIO wiring: BCM10 -> IN1+IN3 and BCM9 -> IN2+IN4.");
	warnIfStartFailed(
		"Slide-1 actuator",
		front_slider_axis.start(),
		"Check DRV8833 #3 direct GPIO wiring: BCM5 -> IN1+IN3 and BCM6 -> IN2+IN4.");
	warnIfStartFailed(
		"Slide-2 actuator",
		rear_slide_axis.start(),
		"Check DRV8833 #4 direct GPIO wiring: BCM12 -> IN1+IN3 and BCM13 -> IN2+IN4.");
	warnIfStartFailed(
		"Lift-2 actuator",
		rear_lift_axis.start(),
		"Check DRV8833 #2 direct GPIO wiring: BCM11 -> IN1+IN3 and BCM7 -> IN2+IN4.");

	std::unique_ptr<IImuSensor> imu_sensor;
	auto hardware_imu = std::make_unique<ImuSensor>();
	if (hardware_imu->start() &&
		WaitForValidImuSample(*hardware_imu, std::chrono::milliseconds(RobotConfig::Sensors::SENSOR_STALE_MS)))
	{
		Logger::info("Hardware IMU online.");
		imu_sensor = std::move(hardware_imu);
	}
	else
	{
		if (hardware_imu)
		{
			hardware_imu->stop();
		}

		Logger::warn("IMU is optional. Falling back to a fixed neutral pose sensor.");
		imu_sensor = std::make_unique<FixedPoseImuSensor>();
	}

	return RunRobotController(
		front_drive,
		middle_drive,
		first_lift_axis,
		front_slider_axis,
		rear_slide_axis,
		rear_lift_axis,
		front_sensor,
		front_downward_sensor,
		&middle_support_sensor,
		&rear_support_sensor,
		*imu_sensor,
		slide1_lower_limit);
}
}

int main()
{
	Logger::info("Climbing robot Linux hardware stack starting.");
	return RunHardwareBringup();
}
