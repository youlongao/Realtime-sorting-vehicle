#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <cerrno>

#include "config.h"

#include <gpiod.h>

namespace
{
constexpr const char* kGpioChip = "/dev/gpiochip0";
constexpr unsigned int kLift1In1In3Gpio = RobotConfig::MotorGPIO::LIFT_1_IN1_IN3; // Physical pin 19
constexpr unsigned int kLift1In2In4Gpio = RobotConfig::MotorGPIO::LIFT_1_IN2_IN4; // Physical pin 21
constexpr unsigned int kLift2In1In3Gpio = RobotConfig::MotorGPIO::LIFT_2_IN1_IN3; // Physical pin 23
constexpr unsigned int kLift2In2In4Gpio = RobotConfig::MotorGPIO::LIFT_2_IN2_IN4; // Physical pin 26

std::string FormatErrno(const std::string& prefix)
{
	return prefix + ": " + std::strerror(errno);
}

enum class TestTarget
{
	Lift1,
	Lift2,
	Both
};

enum class DriveMode
{
	Coast,
	Forward,
	Reverse,
	Brake
};

struct LiftRig
{
	std::string name;
	unsigned int gpio_in1_in3;
	unsigned int gpio_in2_in4;
	gpiod_line_request* request{nullptr};
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
		<< " [lift1|lift2|both] [forward_ms] [reverse_ms] [cycles]\n\n"
		<< "Default:\n"
		<< "  target     = both\n"
		<< "  forward_ms = 800\n"
		<< "  reverse_ms = 800\n"
		<< "  cycles     = 1\n\n"
		<< "Direct GPIO wiring assumptions (no PCA9685 used):\n"
		<< "  Lift-1 DRV8833 (paralleled):\n"
		<< "    BCM10 -> IN1 + IN3\n"
		<< "    BCM9  -> IN2 + IN4\n"
		<< "  Lift-2 DRV8833 (paralleled):\n"
		<< "    BCM11 -> IN1 + IN3\n"
		<< "    BCM7  -> IN2 + IN4\n"
		<< "  OUT1+OUT3 / OUT2+OUT4 -> corresponding lift motor terminals\n"
		<< "  Battery + -> DRV8833 VCC/VM\n"
		<< "  Battery - -> DRV8833 GND and Raspberry Pi GND\n"
		<< "  EEP/nSLEEP must be tied high (3.3V)\n\n"
		<< "Truth table used by this test:\n"
		<< "  Forward : IN1/3=1, IN2/4=0\n"
		<< "  Reverse : IN1/3=0, IN2/4=1\n"
		<< "  Coast   : IN1/3=0, IN2/4=0\n"
		<< "  Brake   : IN1/3=1, IN2/4=1\n";
}

bool ApplyMode(LiftRig& rig, const DriveMode mode)
{
	gpiod_line_value value_a = GPIOD_LINE_VALUE_INACTIVE;
	gpiod_line_value value_b = GPIOD_LINE_VALUE_INACTIVE;

	switch (mode)
	{
	case DriveMode::Forward:
		value_a = GPIOD_LINE_VALUE_ACTIVE;
		value_b = GPIOD_LINE_VALUE_INACTIVE;
		break;
	case DriveMode::Reverse:
		value_a = GPIOD_LINE_VALUE_INACTIVE;
		value_b = GPIOD_LINE_VALUE_ACTIVE;
		break;
	case DriveMode::Brake:
		value_a = GPIOD_LINE_VALUE_ACTIVE;
		value_b = GPIOD_LINE_VALUE_ACTIVE;
		break;
	case DriveMode::Coast:
	default:
		value_a = GPIOD_LINE_VALUE_INACTIVE;
		value_b = GPIOD_LINE_VALUE_INACTIVE;
		break;
	}

	if (gpiod_line_request_set_value(rig.request, rig.gpio_in1_in3, value_a) < 0)
	{
		std::cerr << FormatErrno("Failed to set " + rig.name + " IN1+IN3") << '\n';
		return false;
	}

	if (gpiod_line_request_set_value(rig.request, rig.gpio_in2_in4, value_b) < 0)
	{
		std::cerr << FormatErrno("Failed to set " + rig.name + " IN2+IN4") << '\n';
		return false;
	}

	return true;
}

bool StartRig(gpiod_chip* chip, LiftRig& rig)
{
	auto* settings = gpiod_line_settings_new();
	auto* line_config = gpiod_line_config_new();
	auto* request_config = gpiod_request_config_new();

	if (settings == nullptr || line_config == nullptr || request_config == nullptr)
	{
		std::cerr << "Failed to allocate libgpiod request objects for " << rig.name << ".\n";
		gpiod_line_settings_free(settings);
		gpiod_line_config_free(line_config);
		gpiod_request_config_free(request_config);
		return false;
	}

	gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
	gpiod_line_settings_set_drive(settings, GPIOD_LINE_DRIVE_PUSH_PULL);
	gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);
	gpiod_request_config_set_consumer(request_config, rig.name.c_str());

	const unsigned int offsets[2] = {rig.gpio_in1_in3, rig.gpio_in2_in4};
	gpiod_line_config_add_line_settings(line_config, offsets, 2, settings);
	rig.request = gpiod_chip_request_lines(chip, request_config, line_config);

	gpiod_line_settings_free(settings);
	gpiod_line_config_free(line_config);
	gpiod_request_config_free(request_config);

	if (rig.request == nullptr)
	{
		std::cerr << FormatErrno("Failed to request GPIO lines for " + rig.name) << '\n';
		return false;
	}

	return ApplyMode(rig, DriveMode::Coast);
}

void StopRig(LiftRig& rig)
{
	if (rig.request != nullptr)
	{
		(void)ApplyMode(rig, DriveMode::Coast);
		gpiod_line_request_release(rig.request);
		rig.request = nullptr;
	}
}

bool RunPhase(const std::vector<LiftRig*>& rigs,
			  const DriveMode mode,
			  const int duration_ms,
			  const char* phase_label)
{
	for (LiftRig* rig : rigs)
	{
		if (!ApplyMode(*rig, mode))
		{
			return false;
		}
	}

	std::cout << "[" << phase_label << "] running for " << duration_ms << " ms\n";
	std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));

	for (LiftRig* rig : rigs)
	{
		if (!ApplyMode(*rig, DriveMode::Coast))
		{
			return false;
		}
	}

	return true;
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
	const int forward_ms = (argc >= 3) ? std::atoi(argv[2]) : 800;
	const int reverse_ms = (argc >= 4) ? std::atoi(argv[3]) : 800;
	const int cycles = (argc >= 5) ? std::atoi(argv[4]) : 1;

	if (forward_ms <= 0 || reverse_ms <= 0 || cycles <= 0)
	{
		std::cerr << "forward_ms, reverse_ms, and cycles must be positive integers.\n";
		PrintUsage(argv[0]);
		return 1;
	}

	gpiod_chip* chip = gpiod_chip_open(kGpioChip);
	if (chip == nullptr)
	{
		std::cerr << FormatErrno("Failed to open /dev/gpiochip0") << '\n';
		return 1;
	}

	LiftRig lift1{"lift1", kLift1In1In3Gpio, kLift1In2In4Gpio};
	LiftRig lift2{"lift2", kLift2In1In3Gpio, kLift2In2In4Gpio};

	std::vector<LiftRig*> active_rigs;
	switch (target)
	{
	case TestTarget::Lift1:
		active_rigs.push_back(&lift1);
		break;
	case TestTarget::Lift2:
		active_rigs.push_back(&lift2);
		break;
	case TestTarget::Both:
	default:
		active_rigs.push_back(&lift1);
		active_rigs.push_back(&lift2);
		break;
	}

	for (LiftRig* rig : active_rigs)
	{
		if (!StartRig(chip, *rig))
		{
			for (LiftRig* started_rig : active_rigs)
			{
				StopRig(*started_rig);
			}
			gpiod_chip_close(chip);
			return 1;
		}
	}

	for (int cycle = 1; cycle <= cycles; ++cycle)
	{
		std::cout << "[cycle " << cycle << "] forward\n";
		if (!RunPhase(active_rigs, DriveMode::Forward, forward_ms, "forward"))
		{
			break;
		}

		std::cout << "[cycle " << cycle << "] stop\n";
		std::this_thread::sleep_for(std::chrono::milliseconds(500));

		std::cout << "[cycle " << cycle << "] reverse\n";
		if (!RunPhase(active_rigs, DriveMode::Reverse, reverse_ms, "reverse"))
		{
			break;
		}

		std::cout << "[cycle " << cycle << "] stop\n";
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	for (LiftRig* rig : active_rigs)
	{
		StopRig(*rig);
	}

	gpiod_chip_close(chip);
	std::cout << "Direct GPIO DRV8833 lift test complete.\n";
	return 0;
}
