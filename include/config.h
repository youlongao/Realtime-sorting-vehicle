/*
	Three-stage stair-climbing robot global hardware pin configuration and parameter settings
*/

#ifndef CONFIG_H
#define CONFIG_H

#include <cstdint>

namespace RobotConfig
{
	// ==== I2C bus configuration ====
	namespace I2C
	{
		constexpr uint8_t BUS_ID = 1;				// Raspberry Pi default I2C bus
		constexpr uint8_t MCP23017_ADDR = 0x20;		// MCP23017 GPIO expander address
	}

	namespace Platform
	{
		inline constexpr const char* GPIO_CHIP = "/dev/gpiochip0";
		inline constexpr const char* I2C_DEVICE_PREFIX = "/dev/i2c-";
	}

	// ==== Raspberry Pi direct GPIO allocation for the main robot ====
	namespace MotorGPIO
	{
		// Front wheel DRV8833 (dual-channel)
		constexpr int FRONT_L_IN1 = 17;		// Physical pin 11
		constexpr int FRONT_L_IN2 = 18;		// Physical pin 12
		constexpr int FRONT_R_IN3 = 27;		// Physical pin 13
		constexpr int FRONT_R_IN4 = 22;		// Physical pin 15

		// Middle wheel DRV8833 (dual-channel)
		constexpr int MIDDLE_L_IN1 = 23;	// Physical pin 16
		constexpr int MIDDLE_L_IN2 = 24;	// Physical pin 18
		constexpr int MIDDLE_R_IN3 = 25;	// Physical pin 22
		constexpr int MIDDLE_R_IN4 = 8;		// Physical pin 24

		// Paralleled DRV8833 linear actuators
		constexpr int LIFT_1_IN1_IN3 = 10;	// Physical pin 19
		constexpr int LIFT_1_IN2_IN4 = 9;	// Physical pin 21
		constexpr int LIFT_2_IN1_IN3 = 11;	// Physical pin 23
		constexpr int LIFT_2_IN2_IN4 = 7;	// Physical pin 26
		constexpr int SLIDE_2_IN1_IN3 = 12;	// Physical pin 32
		constexpr int SLIDE_2_IN2_IN4 = 13;	// Physical pin 33
		// Logical direction aliases for modules whose physical installation
		// direction differs from the generic IN1/IN2 naming.
		constexpr int LIFT_2_TO_UPPER_GPIO = LIFT_2_IN1_IN3;
		constexpr int LIFT_2_TO_LOWER_GPIO = LIFT_2_IN2_IN4;
		constexpr int SLIDE_2_TO_FRONT_GPIO = SLIDE_2_IN2_IN4;
		constexpr int SLIDE_2_TO_REAR_GPIO = SLIDE_2_IN1_IN3;
	}

	// Direct GPIOs that remain on the Raspberry Pi for non-motor hardware.
	namespace GPIO
	{
		constexpr int ULTRASONIC_TRIG = 16;	// Physical pin 36
		constexpr int ULTRASONIC_ECHO = 26;	// Physical pin 37
		constexpr unsigned int FRONT_DOWNWARD_DO = 4;	// Physical pin 7
		constexpr unsigned int MIDDLE_SUPPORT_DO = 5;	// Physical pin 29
		constexpr unsigned int REAR_SUPPORT_DO = 21;	// Physical pin 40
		constexpr unsigned int MCP23017_INTA = 19;	// Physical pin 35 — port-A interrupt
		constexpr unsigned int MCP23017_INTB = 20;	// Physical pin 38 — port-B interrupt
	}

	// Legacy direct-input defaults retained so the old standalone GPIO-input
	// helper classes continue to compile, even though the main robot now uses
	// MCP23017 for these signals.
	namespace CompatibilityGPIO
	{
		constexpr int DOWNWARD_DETECTOR = 12;
	}

	// ==== MCP23017 input allocation ====
	namespace MCP23017
	{
		constexpr int POLL_INTERVAL_MS = 10;

		// Legacy MCP23017 TCRT5000 digital input allocation retained for MCP
		// diagnostic tools only. The main robot now reads the three downward
		// sensors directly from Raspberry Pi GPIO::FRONT/MIDDLE/REAR_* pins.
		constexpr uint8_t FRONT_DOWNWARD_DO = 0;	// GPA0
		constexpr uint8_t MIDDLE_SUPPORT_DO = 1;	// GPA1
		constexpr uint8_t REAR_SUPPORT_DO = 2;		// GPA2

		// Limit switches, wired to ground and read with pull-ups enabled
		constexpr uint8_t LIFT_1_UPPER_LIMIT = 3;	// GPA3
		constexpr uint8_t LIFT_1_LOWER_LIMIT = 4;	// GPA4
		constexpr uint8_t SLIDE_2_UPPER_LIMIT = 7;	// GPA7
		constexpr uint8_t SLIDE_2_LOWER_LIMIT = 8;	// GPB0
		constexpr uint8_t LIFT_2_UPPER_LIMIT = 9;	// GPB1
		constexpr uint8_t LIFT_2_LOWER_LIMIT = 10;	// GPB2
	}

	namespace Sensors
	{
		// TCRT5000 DO threshold is adjusted on the sensor module potentiometer.
		// These flags only describe which DO logic level means "surface detected".
		constexpr bool DOWNWARD_ACTIVE_ON_SURFACE = false;
		constexpr bool FRONT_DOWNWARD_ACTIVE_ON_SURFACE = DOWNWARD_ACTIVE_ON_SURFACE;
		constexpr bool MIDDLE_SUPPORT_ACTIVE_ON_SURFACE = DOWNWARD_ACTIVE_ON_SURFACE;
		constexpr bool REAR_SUPPORT_ACTIVE_ON_SURFACE = DOWNWARD_ACTIVE_ON_SURFACE;
		constexpr int DOWNWARD_DEBOUNCE_US = 2500;
		constexpr int ECHO_TIMEOUT_US = 30000;
		constexpr int ECHO_EVENT_BUFFER_SIZE = 8;
		constexpr int GPIO_EVENT_BUFFER_SIZE = 16;
		constexpr float SPEED_OF_SOUND_MPS = 343.0f;
		constexpr float STEP_FACE_MIN_DISTANCE_M = 0.02f;
		constexpr float STEP_FACE_MAX_DISTANCE_M = 0.25f;
		constexpr float READY_TO_CLIMB_DISTANCE_M = 0.07f;
		constexpr int APPROACH_CLOSE_CONFIRM_SAMPLES = 3;
		constexpr float STEP_COMPLETION_CLEARANCE_M = 0.18f;
		constexpr int SENSOR_STALE_MS = 250;
		// Event-driven digital downward / support sensors only push a new
		// reading on pin changes.  A stable pin is therefore perfectly fresh;
		// use a much wider stale window so that a sensor holding its state for
		// several seconds is not discarded.
		constexpr int DOWNWARD_SENSOR_STALE_MS = 10000;
		constexpr int FRONT_DISTANCE_STARTUP_TIMEOUT_MS = 3000;
		// Number of consecutive invalid HC-SR04 readings before the safety
		// manager declares a FrontDistanceTimeout fault.  A single missed echo
		// is normal (the sensor occasionally gets no return); requiring several
		// consecutive failures avoids false-positives from transient no-echo events.
		constexpr int FRONT_DISTANCE_MAX_CONSECUTIVE_INVALID = 3;
	}

	namespace Geometry
	{
		constexpr float SLIDER_MAX_TRAVEL_M = 0.18f;
		constexpr float BODY_LIFT_MAX_TRAVEL_M = 0.20f;
		constexpr float POSITION_TOLERANCE_M = 0.005f;
	}

	namespace Limits
	{
		constexpr bool ACTIVE_LOW = true;
	}

	namespace Motion
	{
		constexpr float APPROACH_SPEED = 0.35f;
		constexpr float CREEP_SPEED = 0.18f;
		constexpr float BODY_LIFT_SPEED = 0.45f;
		constexpr float BODY_LOWER_SPEED = -0.30f;
		constexpr float SLIDER_HOME_SPEED = -0.25f;
		constexpr float HOLD_DEADBAND_M = 0.003f;
		// Maximum time (seconds) to wait for a downward sensor to confirm landing.
		// If the sensor never sees the required drop-then-surface sequence within
		// this window the coordinator raises a fault rather than waiting forever.
		constexpr int SENSOR_CONFIRM_TIMEOUT_S = 30;
		// Maximum time (seconds) to detect a step while driving forward in
		// ApproachingStep.  If the ultrasonic sensor never reads a step face
		// within the band, the coordinator raises an actuator fault so the robot
		// does not drive indefinitely.
		constexpr int APPROACH_TIMEOUT_S = 60;
		// Maximum number of stair steps to climb before transitioning to
		// MotionState::Completed.  Set to 0 for continuous climbing (robot runs
		// until an external stop signal is received).
		constexpr int MAX_CLIMB_CYCLES = 0;
		// After the front downward sensor confirms that the front wheels have
		// reached the step surface, keep lifting briefly so the front assembly
		// fully clears and settles onto the stair before raising the middle.
		constexpr int FRONT_LANDING_EXTRA_LIFT_S = 7;
		// After a downward sensor confirms landing, keep the front and middle
		// drive wheels moving briefly so the section fully rolls onto the stair
		// surface before the next actuator phase.
		constexpr int FRONT_LANDING_EXTRA_DRIVE_MS = 1000;
		constexpr int MIDDLE_LANDING_EXTRA_DRIVE_MS = 100;
		// Maximum time (seconds) to wait for a slider limit switch to confirm.
		constexpr int SLIDER_CONFIRM_TIMEOUT_S = 30;
		// Maximum time (seconds) to wait for the slow high-torque lift modules.
		constexpr int LIFT_CONFIRM_TIMEOUT_S = 180;
	}

	namespace Safety
	{
		constexpr float MAX_SAFE_PITCH_DEG = 25.0f;
		constexpr float MAX_SAFE_ROLL_DEG = 15.0f;
		constexpr int FAULT_LATCH_MS = 500;
	}

	// ==== Real-time and control parameters ====
	namespace Realtime
	{
		constexpr int ULTRASONIC_POLLING_MS = 50;	// Ultrasonic sampling period (20Hz)
		constexpr int SENSOR_TASK_PRIORITY = 80;
		constexpr int MOTION_TASK_PRIORITY = 70;
	}
}

#endif
