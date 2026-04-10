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
		constexpr uint8_t PCA9685_ADDR = 0x40;		// Standalone PCA9685 test board address
		constexpr uint8_t MCP23017_ADDR = 0x20;		// MCP23017 GPIO expander address
		constexpr int PWM_FREQ = 1000;				// Shared PCA PWM frequency for standalone motor tests
	}

	namespace IMU
	{
		constexpr uint8_t ADDRESS = 0x68;			// MPU6050 device address
		constexpr uint8_t POWER_MGMT_REG = 0x6B;	// Wake-up register
		constexpr uint8_t ACCEL_START_REG = 0x3B;	// First accelerometer register
		constexpr float ACCEL_SCALE = 16384.0f;		// LSB per g at +/-2g
		constexpr float GYRO_SCALE = 131.0f;		// LSB per deg/s at +/-250deg/s
		constexpr float COMPLEMENTARY_ALPHA = 0.98f;
	}

	namespace Platform
	{
		inline constexpr const char* GPIO_CHIP = "/dev/gpiochip0";
		inline constexpr const char* I2C_DEVICE_PREFIX = "/dev/i2c-";
	}

	namespace PWM
	{
		constexpr float PCA9685_OSCILLATOR_HZ = 25'000'000.0F;
		constexpr float PCA9685_RESOLUTION = 4096.0F;
	}

	// ==== Standalone PCA9685 diagnostic channel allocation (0-15) ====
	// These channels are retained for independent PCA test tools only.
	namespace PWM_Channels
	{
		constexpr uint8_t FRONT_L_IN1 = 0;
		constexpr uint8_t FRONT_L_IN2 = 1;
		constexpr uint8_t FRONT_R_IN3 = 2;
		constexpr uint8_t FRONT_R_IN4 = 3;
		constexpr uint8_t MIDDLE_L_IN1 = 4;
		constexpr uint8_t MIDDLE_L_IN2 = 5;
		constexpr uint8_t MIDDLE_R_IN3 = 6;
		constexpr uint8_t MIDDLE_R_IN4 = 7;
		constexpr uint8_t LIFT_1_IN1_IN3 = 8;
		constexpr uint8_t LIFT_1_IN2_IN4 = 9;
		constexpr uint8_t LIFT_2_IN1_IN3 = 10;
		constexpr uint8_t LIFT_2_IN2_IN4 = 11;
		constexpr uint8_t SLIDE_1_IN1_IN3 = 12;
		constexpr uint8_t SLIDE_1_IN2_IN4 = 13;
		constexpr uint8_t SLIDE_2_IN1_IN3 = 14;
		constexpr uint8_t SLIDE_2_IN2_IN4 = 15;
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
		constexpr int SLIDE_1_IN1_IN3 = 5;	// Physical pin 29
		constexpr int SLIDE_1_IN2_IN4 = 6;	// Physical pin 31
		constexpr int SLIDE_2_IN1_IN3 = 12;	// Physical pin 32
		constexpr int SLIDE_2_IN2_IN4 = 13;	// Physical pin 33
	}

	// Direct GPIOs that remain on the Raspberry Pi for non-motor hardware.
	namespace GPIO
	{
		constexpr int ULTRASONIC_TRIG = 16;	// Physical pin 36
		constexpr int ULTRASONIC_ECHO = 26;	// Physical pin 37
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

		// TCRT5000 digital outputs (DO only; AO is unused)
		constexpr uint8_t FRONT_DOWNWARD_DO = 0;	// GPA0
		constexpr uint8_t MIDDLE_SUPPORT_DO = 1;	// GPA1
		constexpr uint8_t REAR_SUPPORT_DO = 2;		// GPA2

		// Limit switches, wired to ground and read with pull-ups enabled
		constexpr uint8_t LIFT_1_UPPER_LIMIT = 3;	// GPA3
		constexpr uint8_t LIFT_1_LOWER_LIMIT = 4;	// GPA4
		constexpr uint8_t SLIDE_1_UPPER_LIMIT = 5;	// GPA5
		constexpr uint8_t SLIDE_1_LOWER_LIMIT = 6;	// GPA6
		constexpr uint8_t SLIDE_2_UPPER_LIMIT = 7;	// GPA7
		constexpr uint8_t SLIDE_2_LOWER_LIMIT = 8;	// GPB0
		constexpr uint8_t LIFT_2_UPPER_LIMIT = 9;	// GPB1
		constexpr uint8_t LIFT_2_LOWER_LIMIT = 10;	// GPB2
	}

	namespace Sensors
	{
		constexpr bool DOWNWARD_ACTIVE_ON_SURFACE = true;
		constexpr int DOWNWARD_DEBOUNCE_US = 2500;
		constexpr int ECHO_TIMEOUT_US = 30000;
		constexpr int ECHO_EVENT_BUFFER_SIZE = 8;
		constexpr int GPIO_EVENT_BUFFER_SIZE = 16;
		constexpr float SPEED_OF_SOUND_MPS = 343.0f;
		constexpr float STEP_FACE_MIN_DISTANCE_M = 0.04f;
		constexpr float STEP_FACE_MAX_DISTANCE_M = 0.25f;
		constexpr float READY_TO_CLIMB_DISTANCE_M = 0.12f;
		constexpr float STEP_COMPLETION_CLEARANCE_M = 0.18f;
		constexpr int SENSOR_STALE_MS = 250;
	}

	namespace Geometry
	{
		constexpr float SLIDER_HOME_OFFSET_M = 0.0f;
		constexpr float SLIDER_MAX_TRAVEL_M = 0.18f;
		constexpr float BODY_LIFT_MAX_TRAVEL_M = 0.20f;
		constexpr float POSITION_TOLERANCE_M = 0.005f;
	}

	namespace Servo
	{
		constexpr float MIN_ANGLE_DEG = 0.0F;
		constexpr float MAX_ANGLE_DEG = 180.0F;
		constexpr float MIN_PULSE_WIDTH_US = 500.0F;
		constexpr float MAX_PULSE_WIDTH_US = 2500.0F;
		constexpr float SAFE_ANGLE_DEG = 90.0F;
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
		constexpr int IMU_POLLINGS_MS = 10;			// IMU data sampling period (100Hz)
		constexpr int ULTRASONIC_POLLING_MS = 50;	// Ultrasonic sampling period (20Hz)
		constexpr int SENSOR_TASK_PRIORITY = 80;
		constexpr int MOTION_TASK_PRIORITY = 70;
	}
}

#endif
