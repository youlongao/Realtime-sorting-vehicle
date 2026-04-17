#include <array>
#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include "config.h"
#include "mcp23017_driver.h"

namespace
{
struct LimitInput
{
	const char* name;
	std::uint8_t pin;
	const char* mcp_pin_name;
};

constexpr std::array<LimitInput, 8> kLimitInputs{{
	{"lift1_upper", RobotConfig::MCP23017::LIFT_1_UPPER_LIMIT, "GPA3"},
	{"lift1_lower", RobotConfig::MCP23017::LIFT_1_LOWER_LIMIT, "GPA4"},
	{"slide2_upper", RobotConfig::MCP23017::SLIDE_2_UPPER_LIMIT, "GPA7"},
	{"slide2_lower", RobotConfig::MCP23017::SLIDE_2_LOWER_LIMIT, "GPB0"},
	{"lift2_upper", RobotConfig::MCP23017::LIFT_2_UPPER_LIMIT, "GPB1"},
	{"lift2_lower", RobotConfig::MCP23017::LIFT_2_LOWER_LIMIT, "GPB2"},
}};

void PrintUsage(const char* program_name)
{
	std::cout
		<< "Usage:\n  " << program_name << " [samples] [interval_ms]\n\n"
		<< "Default:\n"
		<< "  samples     = 100\n"
		<< "  interval_ms = 200\n\n"
		<< "Meaning:\n"
		<< "  raw=1 means MCP23017 input is high from pull-up/open switch.\n"
		<< "  triggered=1 means active-low switch is pressed/connected to GND.\n\n"
		<< "Expected wiring for each mechanical limit switch:\n"
		<< "  NO  -> MCP23017 input pin\n"
		<< "  COM -> GND\n\n"
		<< "Limit mapping used by config.h:\n";

	for (const auto& input : kLimitInputs)
	{
		std::cout << "  " << input.name << " -> " << input.mcp_pin_name
				  << " / pin " << static_cast<int>(input.pin) << '\n';
	}
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

	const int samples = (argc >= 2) ? std::atoi(argv[1]) : 100;
	const int interval_ms = (argc >= 3) ? std::atoi(argv[2]) : 200;
	if (samples <= 0 || interval_ms <= 0)
	{
		std::cerr << "samples and interval_ms must be positive integers.\n";
		PrintUsage(argv[0]);
		return 1;
	}

	Robot::Mcp23017Driver driver;
	if (!driver.start())
	{
		std::cerr << "Failed to start MCP23017. Check SDA/SCL/VDD/VSS, RESET high, and address 0x20.\n";
		return 1;
	}

	for (const auto& input : kLimitInputs)
	{
		if (!driver.configureInput(input.pin, true))
		{
			std::cerr << "Failed to configure MCP23017 pin " << static_cast<int>(input.pin)
					  << " for " << input.name << ".\n";
			return 1;
		}
	}

	std::cout << "Starting MCP23017 limit switch test\n";
	PrintUsage(argv[0]);
	std::cout << "\nPress one limit switch at a time. A changed input is marked with CHANGED.\n";

	std::array<bool, kLimitInputs.size()> previous_raw{};
	std::array<bool, kLimitInputs.size()> previous_valid{};

	for (int sample = 1; sample <= samples; ++sample)
	{
		std::cout << "[sample " << sample << "] ";
		for (std::size_t index = 0; index < kLimitInputs.size(); ++index)
		{
			const auto& input = kLimitInputs[index];
			bool raw = false;
			if (!driver.readPin(input.pin, raw))
			{
				std::cout << input.name << "=read_error ";
				previous_valid[index] = false;
				continue;
			}

			const bool triggered = !raw;
			const bool changed = previous_valid[index] && previous_raw[index] != raw;
			previous_raw[index] = raw;
			previous_valid[index] = true;

			std::cout << input.name << "/" << input.mcp_pin_name
					  << "(raw=" << (raw ? 1 : 0)
					  << ",triggered=" << (triggered ? 1 : 0);
			if (changed)
			{
				std::cout << ",CHANGED";
			}
			std::cout << ") ";
		}
		std::cout << '\n';
		std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
	}

	std::cout << "MCP23017 limit switch test complete.\n";
	return 0;
}
