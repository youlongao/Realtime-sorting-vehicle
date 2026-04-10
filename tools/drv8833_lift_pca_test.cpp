#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "config.h"
#include "pca9685_driver.h"

using namespace Robot;

namespace
{
enum class TestTarget
{
	Lift1,
	Lift2,
	Both
};

struct LiftChannels
{
	unsigned int forward_channel;
	unsigned int reverse_channel;
};

TestTarget ParseTarget(const std::string& value)
{
	if (value == "lift1")
	{
		return TestTarget::Lift1;
	}

	if (value == "lift2")
	{
		return TestTarget::Lift2;
	}

	return TestTarget::Both;
}

void PrintUsage(const char* program_name)
{
	std::cout
		<< "Usage:\n  " << program_name
		<< " [lift1|lift2|both] [speed] [forward_ms] [reverse_ms] [cycles]\n\n"
		<< "Default:\n"
		<< "  target     = both\n"
		<< "  speed      = 0.20\n"
		<< "  forward_ms = 800\n"
		<< "  reverse_ms = 800\n"
		<< "  cycles     = 1\n\n"
		<< "Standalone PCA9685 wiring assumptions:\n"
		<< "  Lift-1 DRV8833 (paralleled): CH8 -> IN1+IN3, CH9 -> IN2+IN4\n"
		<< "  Lift-2 DRV8833 (paralleled): CH10 -> IN1+IN3, CH11 -> IN2+IN4\n";
}

bool Coast(Pca9685Driver& pca, const LiftChannels& channels)
{
	return pca.disableChannel(static_cast<std::uint8_t>(channels.forward_channel)) &&
		   pca.disableChannel(static_cast<std::uint8_t>(channels.reverse_channel));
}

bool ApplyCommand(Pca9685Driver& pca, const LiftChannels& channels, const float command)
{
	if (command > 0.0F)
	{
		return pca.setDutyCycle(static_cast<std::uint8_t>(channels.forward_channel), command) &&
			   pca.disableChannel(static_cast<std::uint8_t>(channels.reverse_channel));
	}

	if (command < 0.0F)
	{
		return pca.disableChannel(static_cast<std::uint8_t>(channels.forward_channel)) &&
			   pca.setDutyCycle(static_cast<std::uint8_t>(channels.reverse_channel), -command);
	}

	return Coast(pca, channels);
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
	const float speed = (argc >= 3) ? std::stof(argv[2]) : 0.20F;
	const int forward_ms = (argc >= 4) ? std::atoi(argv[3]) : 800;
	const int reverse_ms = (argc >= 5) ? std::atoi(argv[4]) : 800;
	const int cycles = (argc >= 6) ? std::atoi(argv[5]) : 1;

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

	const LiftChannels lift1{
		RobotConfig::PWM_Channels::LIFT_1_IN1_IN3,
		RobotConfig::PWM_Channels::LIFT_1_IN2_IN4};
	const LiftChannels lift2{
		RobotConfig::PWM_Channels::LIFT_2_IN1_IN3,
		RobotConfig::PWM_Channels::LIFT_2_IN2_IN4};

	(void)Coast(*pwm_driver, lift1);
	(void)Coast(*pwm_driver, lift2);

	for (int cycle = 1; cycle <= cycles; ++cycle)
	{
		std::cout << "[cycle " << cycle << "] forward\n";
		if (target == TestTarget::Lift1 || target == TestTarget::Both)
		{
			(void)ApplyCommand(*pwm_driver, lift1, speed);
		}
		if (target == TestTarget::Lift2 || target == TestTarget::Both)
		{
			(void)ApplyCommand(*pwm_driver, lift2, speed);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(forward_ms));

		std::cout << "[cycle " << cycle << "] stop\n";
		(void)Coast(*pwm_driver, lift1);
		(void)Coast(*pwm_driver, lift2);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));

		std::cout << "[cycle " << cycle << "] reverse\n";
		if (target == TestTarget::Lift1 || target == TestTarget::Both)
		{
			(void)ApplyCommand(*pwm_driver, lift1, -speed);
		}
		if (target == TestTarget::Lift2 || target == TestTarget::Both)
		{
			(void)ApplyCommand(*pwm_driver, lift2, -speed);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(reverse_ms));

		std::cout << "[cycle " << cycle << "] stop\n";
		(void)Coast(*pwm_driver, lift1);
		(void)Coast(*pwm_driver, lift2);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	(void)Coast(*pwm_driver, lift1);
	(void)Coast(*pwm_driver, lift2);
	std::cout << "PCA9685 DRV8833 lift test complete.\n";
	return 0;
}
