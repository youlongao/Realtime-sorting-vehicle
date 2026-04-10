#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "pca9685_driver.h"

using namespace Robot;

namespace
{
void PrintUsage(const char* program_name)
{
	std::cout
		<< "Usage:\n  " << program_name
		<< " [forward_channel] [reverse_channel] [speed] [forward_ms] [reverse_ms] [cycles]\n\n"
		<< "Default:\n"
		<< "  forward_channel = 4\n"
		<< "  reverse_channel = 5\n"
		<< "  speed           = 1.00\n"
		<< "  forward_ms      = 1500\n"
		<< "  reverse_ms      = 1500\n"
		<< "  cycles          = 3\n\n"
		<< "Recommended minimal wiring for PCA -> DRV8833 -> one motor:\n"
		<< "  Raspberry Pi 3.3V -> PCA9685 VCC\n"
		<< "  Raspberry Pi GND  -> PCA9685 GND\n"
		<< "  Raspberry Pi SDA  -> PCA9685 SDA\n"
		<< "  Raspberry Pi SCL  -> PCA9685 SCL\n"
		<< "  PCA9685 OE        -> GND\n"
		<< "  PCA channel A     -> DRV8833 IN1 + IN3\n"
		<< "  PCA channel B     -> DRV8833 IN2 + IN4\n"
		<< "  DRV8833 OUT1+OUT3 -> Motor terminal A\n"
		<< "  DRV8833 OUT2+OUT4 -> Motor terminal B\n"
		<< "  DRV8833 EEP/nSLEEP -> 3.3V\n"
		<< "  Battery +         -> DRV8833 VCC/VM\n"
		<< "  Battery -         -> DRV8833 GND and Raspberry Pi GND\n\n"
		<< "This test bypasses the main robot logic and only verifies one PCA-controlled motor channel pair.\n";
}

bool Coast(Pca9685Driver& pca, const unsigned int forward_channel, const unsigned int reverse_channel)
{
	return pca.disableChannel(static_cast<std::uint8_t>(forward_channel)) &&
		   pca.disableChannel(static_cast<std::uint8_t>(reverse_channel));
}

bool RunDirection(Pca9685Driver& pca,
				  const unsigned int active_channel,
				  const unsigned int inactive_channel,
				  const float speed)
{
	return pca.disableChannel(static_cast<std::uint8_t>(inactive_channel)) &&
		   pca.setDutyCycle(static_cast<std::uint8_t>(active_channel), speed);
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

	const unsigned int forward_channel = (argc >= 2) ? static_cast<unsigned int>(std::strtoul(argv[1], nullptr, 10)) : 4U;
	const unsigned int reverse_channel = (argc >= 3) ? static_cast<unsigned int>(std::strtoul(argv[2], nullptr, 10)) : 5U;
	const float speed = (argc >= 4) ? std::stof(argv[3]) : 1.0F;
	const int forward_ms = (argc >= 5) ? std::atoi(argv[4]) : 1500;
	const int reverse_ms = (argc >= 6) ? std::atoi(argv[5]) : 1500;
	const int cycles = (argc >= 7) ? std::atoi(argv[6]) : 3;

	if (forward_channel >= 16U || reverse_channel >= 16U || forward_channel == reverse_channel)
	{
		std::cerr << "Channel numbers must be different and in range [0, 15].\n";
		return 1;
	}

	if (speed <= 0.0F || speed > 1.0F || forward_ms <= 0 || reverse_ms <= 0 || cycles <= 0)
	{
		std::cerr << "Invalid arguments. speed must be in (0, 1], and all durations/cycles must be positive.\n";
		PrintUsage(argv[0]);
		return 1;
	}

	auto pca = std::make_unique<Pca9685Driver>();
	if (!pca->start())
	{
		std::cerr << "Failed to start PCA9685. Check VCC/GND/SDA/SCL wiring, address 0x40, and OE tied low.\n";
		return 1;
	}

	if (!Coast(*pca, forward_channel, reverse_channel))
	{
		std::cerr << "Failed to place PCA channels into coast mode before the test.\n";
		return 1;
	}

	std::cout << "Starting PCA -> DRV8833 single-motor test\n";
	std::cout << "Forward channel = CH" << forward_channel << '\n';
	std::cout << "Reverse channel = CH" << reverse_channel << '\n';
	std::cout << "Speed           = " << speed << '\n';
	std::cout << "Forward time    = " << forward_ms << " ms\n";
	std::cout << "Reverse time    = " << reverse_ms << " ms\n";
	std::cout << "Cycles          = " << cycles << '\n';

	for (int cycle = 1; cycle <= cycles; ++cycle)
	{
		std::cout << "[cycle " << cycle << "] forward\n";
		if (!RunDirection(*pca, forward_channel, reverse_channel, speed))
		{
			std::cerr << "Failed to drive forward using PCA channels.\n";
			(void)Coast(*pca, forward_channel, reverse_channel);
			return 1;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(forward_ms));

		std::cout << "[cycle " << cycle << "] stop\n";
		(void)Coast(*pca, forward_channel, reverse_channel);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));

		std::cout << "[cycle " << cycle << "] reverse\n";
		if (!RunDirection(*pca, reverse_channel, forward_channel, speed))
		{
			std::cerr << "Failed to drive reverse using PCA channels.\n";
			(void)Coast(*pca, forward_channel, reverse_channel);
			return 1;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(reverse_ms));

		std::cout << "[cycle " << cycle << "] stop\n";
		(void)Coast(*pca, forward_channel, reverse_channel);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	(void)Coast(*pca, forward_channel, reverse_channel);
	std::cout << "PCA -> DRV8833 single-motor test complete.\n";
	return 0;
}
