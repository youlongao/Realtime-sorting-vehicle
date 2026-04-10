#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "config.h"
#include "pca9685_driver.h"

using namespace Robot;

namespace
{
enum class TestTarget
{
	Front,
	Middle,
	Both
};

struct WheelBoardChannels
{
	unsigned int left_in1;
	unsigned int left_in2;
	unsigned int right_in3;
	unsigned int right_in4;
};

TestTarget ParseTarget(const std::string& value)
{
	if (value == "front")
	{
		return TestTarget::Front;
	}

	if (value == "middle")
	{
		return TestTarget::Middle;
	}

	return TestTarget::Both;
}

void PrintUsage(const char* program_name)
{
	std::cout
		<< "Usage:\n  " << program_name
		<< " [front|middle|both] [speed] [forward_ms] [reverse_ms] [cycles]\n\n"
		<< "Default:\n"
		<< "  target     = both\n"
		<< "  speed      = 0.35\n"
		<< "  forward_ms = 1500\n"
		<< "  reverse_ms = 1500\n"
		<< "  cycles     = 3\n\n"
		<< "Standalone PCA9685 wiring assumptions:\n"
		<< "  Front wheel board: CH0/CH1/CH2/CH3 -> IN1/IN2/IN3/IN4\n"
		<< "  Middle wheel board: CH4/CH5/CH6/CH7 -> IN1/IN2/IN3/IN4\n"
		<< "  Each DRV8833 board must have EEP/nSLEEP tied high\n";
}

bool Coast(Pca9685Driver& pca, const WheelBoardChannels& channels)
{
	return pca.disableChannel(static_cast<std::uint8_t>(channels.left_in1)) &&
		   pca.disableChannel(static_cast<std::uint8_t>(channels.left_in2)) &&
		   pca.disableChannel(static_cast<std::uint8_t>(channels.right_in3)) &&
		   pca.disableChannel(static_cast<std::uint8_t>(channels.right_in4));
}

bool ApplySpeed(Pca9685Driver& pca,
				const float speed,
				const unsigned int in1_channel,
				const unsigned int in2_channel)
{
	if (speed > 0.0F)
	{
		return pca.setDutyCycle(static_cast<std::uint8_t>(in1_channel), speed) &&
			   pca.disableChannel(static_cast<std::uint8_t>(in2_channel));
	}

	if (speed < 0.0F)
	{
		return pca.disableChannel(static_cast<std::uint8_t>(in1_channel)) &&
			   pca.setDutyCycle(static_cast<std::uint8_t>(in2_channel), -speed);
	}

	return pca.disableChannel(static_cast<std::uint8_t>(in1_channel)) &&
		   pca.disableChannel(static_cast<std::uint8_t>(in2_channel));
}

bool ApplyBoardCommand(Pca9685Driver& pca, const WheelBoardChannels& channels, const float speed)
{
	return ApplySpeed(pca, speed, channels.left_in1, channels.left_in2) &&
		   ApplySpeed(pca, speed, channels.right_in3, channels.right_in4);
}
}

int main(int argc, char* argv[])
{
	if (argc >= 2)
	{
		const std::string first_arg = argv[1];
		if (first_arg == "-h" || first_arg == "--help")
		{
			PrintUsage(argv[0]);
			return 0;
		}
	}

	const TestTarget target = (argc >= 2) ? ParseTarget(argv[1]) : TestTarget::Both;
	const float speed = (argc >= 3) ? std::stof(argv[2]) : 0.35F;
	const int forward_ms = (argc >= 4) ? std::atoi(argv[3]) : 1500;
	const int reverse_ms = (argc >= 5) ? std::atoi(argv[4]) : 1500;
	const int cycles = (argc >= 6) ? std::atoi(argv[5]) : 3;

	if (speed <= 0.0F || speed > 1.0F || forward_ms <= 0 || reverse_ms <= 0 || cycles <= 0)
	{
		std::cerr << "Invalid arguments. speed must be in (0, 1], and all durations/cycles must be positive.\n";
		PrintUsage(argv[0]);
		return 1;
	}

	auto pwm_driver = std::make_unique<Pca9685Driver>();
	if (!pwm_driver->start())
	{
		std::cerr << "Failed to start PCA9685. Check I2C wiring, address 0x40, and OE tied low.\n";
		return 1;
	}

	const WheelBoardChannels front_channels{
		RobotConfig::PWM_Channels::FRONT_L_IN1,
		RobotConfig::PWM_Channels::FRONT_L_IN2,
		RobotConfig::PWM_Channels::FRONT_R_IN3,
		RobotConfig::PWM_Channels::FRONT_R_IN4};
	const WheelBoardChannels middle_channels{
		RobotConfig::PWM_Channels::MIDDLE_L_IN1,
		RobotConfig::PWM_Channels::MIDDLE_L_IN2,
		RobotConfig::PWM_Channels::MIDDLE_R_IN3,
		RobotConfig::PWM_Channels::MIDDLE_R_IN4};

	(void)Coast(*pwm_driver, front_channels);
	(void)Coast(*pwm_driver, middle_channels);

	for (int cycle = 1; cycle <= cycles; ++cycle)
	{
		std::cout << "[cycle " << cycle << "] forward\n";
		if (target == TestTarget::Front || target == TestTarget::Both)
		{
			(void)ApplyBoardCommand(*pwm_driver, front_channels, speed);
		}
		if (target == TestTarget::Middle || target == TestTarget::Both)
		{
			(void)ApplyBoardCommand(*pwm_driver, middle_channels, speed);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(forward_ms));

		std::cout << "[cycle " << cycle << "] stop\n";
		(void)Coast(*pwm_driver, front_channels);
		(void)Coast(*pwm_driver, middle_channels);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));

		std::cout << "[cycle " << cycle << "] reverse\n";
		if (target == TestTarget::Front || target == TestTarget::Both)
		{
			(void)ApplyBoardCommand(*pwm_driver, front_channels, -speed);
		}
		if (target == TestTarget::Middle || target == TestTarget::Both)
		{
			(void)ApplyBoardCommand(*pwm_driver, middle_channels, -speed);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(reverse_ms));

		std::cout << "[cycle " << cycle << "] stop\n";
		(void)Coast(*pwm_driver, front_channels);
		(void)Coast(*pwm_driver, middle_channels);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	(void)Coast(*pwm_driver, front_channels);
	(void)Coast(*pwm_driver, middle_channels);
	std::cout << "DRV8833 + PCA9685 wheel test complete.\n";
	return 0;
}
