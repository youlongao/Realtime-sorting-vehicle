#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "config.h"
#include "motor_driver.h"

using namespace Robot;

namespace
{
enum class TestTarget
{
	Front,
	Middle,
	Both
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
		<< " [front|middle|both] [forward_ms] [reverse_ms] [cycles]\n\n"
		<< "Default:\n"
		<< "  target     = both\n"
		<< "  forward_ms = 1500\n"
		<< "  reverse_ms = 1500\n"
		<< "  cycles     = 3\n\n"
		<< "Direct GPIO wiring assumptions:\n"
		<< "  Front wheel DRV8833:\n"
		<< "    GPIO " << RobotConfig::MotorGPIO::FRONT_L_IN1 << " -> left IN1\n"
		<< "    GPIO " << RobotConfig::MotorGPIO::FRONT_L_IN2 << " -> left IN2\n"
		<< "    GPIO " << RobotConfig::MotorGPIO::FRONT_R_IN3 << " -> right IN3\n"
		<< "    GPIO " << RobotConfig::MotorGPIO::FRONT_R_IN4 << " -> right IN4\n"
		<< "  Middle wheel DRV8833:\n"
		<< "    GPIO " << RobotConfig::MotorGPIO::MIDDLE_L_IN1 << " -> left logical IN1\n"
		<< "    GPIO " << RobotConfig::MotorGPIO::MIDDLE_L_IN2 << " -> left logical IN2\n"
		<< "    GPIO " << RobotConfig::MotorGPIO::MIDDLE_R_IN3 << " -> right logical IN3\n"
		<< "    GPIO " << RobotConfig::MotorGPIO::MIDDLE_R_IN4 << " -> right logical IN4\n"
		<< "  OUT1/OUT2 -> left wheel motor, OUT3/OUT4 -> right wheel motor\n"
		<< "  Battery + -> DRV8833 VCC/VM\n"
		<< "  Battery - -> DRV8833 GND and Raspberry Pi GND\n"
		<< "  EEP/nSLEEP must be tied high (3.3V)\n";
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
	const int forward_ms = (argc >= 3) ? std::atoi(argv[2]) : 1500;
	const int reverse_ms = (argc >= 4) ? std::atoi(argv[3]) : 1500;
	const int cycles = (argc >= 5) ? std::atoi(argv[4]) : 3;

	if (forward_ms <= 0 || reverse_ms <= 0 || cycles <= 0)
	{
		std::cerr << "forward_ms, reverse_ms, and cycles must be positive integers.\n";
		PrintUsage(argv[0]);
		return 1;
	}

	MotorDriver front_driver(
		"front-wheel-test",
		RobotConfig::MotorGPIO::FRONT_L_IN1,
		RobotConfig::MotorGPIO::FRONT_L_IN2,
		RobotConfig::MotorGPIO::FRONT_R_IN3,
		RobotConfig::MotorGPIO::FRONT_R_IN4);
	MotorDriver middle_driver(
		"middle-wheel-test",
		RobotConfig::MotorGPIO::MIDDLE_L_IN1,
		RobotConfig::MotorGPIO::MIDDLE_L_IN2,
		RobotConfig::MotorGPIO::MIDDLE_R_IN3,
		RobotConfig::MotorGPIO::MIDDLE_R_IN4);

	std::vector<MotorDriver*> active_drivers;
	switch (target)
	{
	case TestTarget::Front:
		active_drivers.push_back(&front_driver);
		break;
	case TestTarget::Middle:
		active_drivers.push_back(&middle_driver);
		break;
	case TestTarget::Both:
	default:
		active_drivers.push_back(&front_driver);
		active_drivers.push_back(&middle_driver);
		break;
	}

	for (MotorDriver* driver : active_drivers)
	{
		if (!driver->start())
		{
			std::cerr << "Failed to start one or more direct wheel drivers.\n";
			return 1;
		}
		driver->stop();
	}

	std::cout << "Starting direct GPIO DRV8833 wheel test\n";
	std::cout << "Forward time  = " << forward_ms << " ms\n";
	std::cout << "Reverse time  = " << reverse_ms << " ms\n";
	std::cout << "Cycles        = " << cycles << '\n';
	std::cout << "Front GPIOs   = "
			  << RobotConfig::MotorGPIO::FRONT_L_IN1 << '/'
			  << RobotConfig::MotorGPIO::FRONT_L_IN2 << " and "
			  << RobotConfig::MotorGPIO::FRONT_R_IN3 << '/'
			  << RobotConfig::MotorGPIO::FRONT_R_IN4 << '\n';
	std::cout << "Middle GPIOs  = "
			  << RobotConfig::MotorGPIO::MIDDLE_L_IN1 << '/'
			  << RobotConfig::MotorGPIO::MIDDLE_L_IN2 << " and "
			  << RobotConfig::MotorGPIO::MIDDLE_R_IN3 << '/'
			  << RobotConfig::MotorGPIO::MIDDLE_R_IN4 << '\n';

	for (int cycle = 1; cycle <= cycles; ++cycle)
	{
		std::cout << "[cycle " << cycle << "] forward\n";
		for (MotorDriver* driver : active_drivers)
		{
			driver->forward(1.0F);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(forward_ms));

		std::cout << "[cycle " << cycle << "] stop\n";
		for (MotorDriver* driver : active_drivers)
		{
			driver->stop();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(500));

		std::cout << "[cycle " << cycle << "] reverse\n";
		for (MotorDriver* driver : active_drivers)
		{
			driver->backward(1.0F);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(reverse_ms));

		std::cout << "[cycle " << cycle << "] stop\n";
		for (MotorDriver* driver : active_drivers)
		{
			driver->stop();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	for (MotorDriver* driver : active_drivers)
	{
		driver->stop();
	}

	std::cout << "Direct GPIO DRV8833 wheel test complete.\n";
	return 0;
}
