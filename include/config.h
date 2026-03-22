/*
	Three-stage stair-climbing robot global hardware pin configuration and parameter settings
*/

#ifndef CONFIG_H
#define CONFIG_H

#include <cstdint>
namespace RobotConfig
{
	// ==== I2C Bus confirguration ===
	namespace I2C 
	{
		constexpr uint8_t BUS_ID = 1;	// pin5 defaults I2C bus
		constexpr uint8_t PCA9685_ADDR = 0x40;	// PCA9685 PWM expansion board address
		constexpr int PWM_FREQ = 50;	// Motor drive PWM frequency (50Hz)
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
	// ==== PCA9685 PWM channel allocate (0-15) ====
	// The speed control signal for the TB6612 and the speed regulation signal for the BTS7960
	namespace PWM_Channels
	{
		// TB6612 drive wheels (front and middle sections; rear support section is passive)
		constexpr uint8_t FRONT_Wheel_L = 0;	// Front Left wheel PWMA
		constexpr uint8_t FRONT_Wheel_R = 1;	// Front Right wheel PWMB
		constexpr uint8_t MIDDLE_Wheel_L = 2;	// Middle Left wheel PWMA
		constexpr uint8_t MIDDLE_Wheel_R = 3;	// Middle Right wheel PWMB

		// BTS7960 Lifting and Sliding module (high-power motor)
		constexpr uint8_t LIFT_1_RPWM = 4;	// Lifting module 1, forward speed regulation
		constexpr uint8_t LIFT_1_LPWM = 5;	// Lifting module 1, reverse speed adjustment
		constexpr uint8_t LIFT_2_RPWM = 6;	// Lifting module 2, forward speed regulation
		constexpr uint8_t LIFT_2_LPWM = 7;	// Lifting module 2, reverse speed adjustment

		constexpr uint8_t SLIDE_1_RPWM = 8;		// Sliding module 1, forward speed regulation
		constexpr uint8_t SLIDE_1_LPWM = 9;		// Sliding module 1, reverse speed adjustment
		constexpr uint8_t SLIDE_2_RPWM = 10;	// Sliding module 2, forward speed regulation
		constexpr uint8_t SLIDE_2_LPWM = 11;	// Sliding module 2, reverse speed adjustment
	}

	// Raspberry Pi GPIO Assignment (libgpio v2)
	namespace GPIO
	{
		// TB6612 direction control pin (AIN1/AIN2, BIN1/BIN2)
		// Front section
		constexpr int FRONT_L_IN1 = 17;		// GPIO 17
		constexpr int FRONT_L_IN2 = 18;		// GPIO 18
		constexpr int FRONT_R_IN1 = 27;		// GPIO 27
		constexpr int FRONT_R_IN2 = 22;		// GPIO 22

		// Middle section (second motorized wheel pair)
		constexpr int MID_L_IN1 = 23;		// GPIO 23
		constexpr int MID_L_IN2 = 24;		// GPIO 24
		constexpr int MID_R_IN1 = 25;		// GPIO 25
		constexpr int MID_R_IN2 = 8;		// GPIO 8

		// Sensors connected
		constexpr int ULTRASONIC_TRIG = 5;			// Ultrasonic tirgger pin
		constexpr int ULTRASONIC_ECHO = 6;			// Ultrasonic echo pin (Thread blocking is required for I/O listening)
		constexpr int DOWNWARD_DETECTOR = 12;		// Front downward-facing sensor
		constexpr int MIDDLE_SUPPORT_DETECTOR = 4;	// Middle support confirmation sensor
		constexpr int REAR_SUPPORT_DETECTOR = 7;	// Rear support confirmation sensor

		// Linear actuator feedback wiring
		constexpr int LIFT_1_UPPER_LIMIT = 10;		// First lift upper limit
		constexpr int LIFT_1_LOWER_LIMIT = 11;		// First lift lower limit
		constexpr int SLIDE_1_UPPER_LIMIT = 13;	// Front slider upper limit
		constexpr int SLIDE_1_LOWER_LIMIT = 16;	// Front slider lower limit
		constexpr int SLIDE_2_UPPER_LIMIT = 19;	// Rear slider upper limit
		constexpr int SLIDE_2_LOWER_LIMIT = 20;	// Rear slider lower limit
		constexpr int LIFT_2_UPPER_LIMIT = 21;		// Rear lift upper limit
		constexpr int LIFT_2_LOWER_LIMIT = 26;		// Rear lift lower limit
	}

	namespace Sensors
	{
		constexpr bool DOWNWARD_ACTIVE_ON_SURFACE = true;
		constexpr int DOWNWARD_DEBOUNCE_US = 2500;
		constexpr int ECHO_TIMEOUT_US = 30000;
		constexpr int ECHO_EVENT_BUFFER_SIZE = 8;
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

	namespace Encoder
	{
		constexpr int EVENT_BUFFER_SIZE = 16;
		constexpr float DEFAULT_METERS_PER_TICK = 0.0005F;
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

	// ==== Real time and control parameters ====
	namespace Realtime
	{	
		// Achieved through thread cycles or timers [cite:213,360]
		constexpr int IMU_POLLINGS_MS = 10;		// IMU data samping period (100Hz)
		constexpr int SENSOR_TASK_PRIORITY = 80;	// Sensor thread real time priority
		constexpr int MOTION_TASK_PRIORITY = 70;	// Motion thread real time priority
	}
}

#endif
