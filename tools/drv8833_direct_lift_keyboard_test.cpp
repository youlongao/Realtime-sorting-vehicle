#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include <cerrno>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "config.h"

#include <gpiod.h>

namespace
{
constexpr const char* kGpioChip = "/dev/gpiochip0";

enum class DriveMode
{
	Coast,
	Forward,
	Reverse
};

struct LiftRig
{
	std::string name;
	unsigned int gpio_in1_in3;
	unsigned int gpio_in2_in4;
	std::chrono::steady_clock::time_point command_deadline{};
	DriveMode current_mode{DriveMode::Coast};
};

class TerminalRawMode
{
public:
	TerminalRawMode()
	{
		if (tcgetattr(STDIN_FILENO, &original_) != 0)
		{
			return;
		}

		termios raw = original_;
		raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
		raw.c_cc[VMIN] = 0;
		raw.c_cc[VTIME] = 0;

		if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0)
		{
			active_ = true;
		}
	}

	~TerminalRawMode()
	{
		if (active_)
		{
			(void)tcsetattr(STDIN_FILENO, TCSANOW, &original_);
		}
	}

	TerminalRawMode(const TerminalRawMode&) = delete;
	TerminalRawMode& operator=(const TerminalRawMode&) = delete;

private:
	termios original_{};
	bool active_{false};
};

std::string FormatErrno(const std::string& prefix)
{
	return prefix + ": " + std::strerror(errno);
}

void PrintUsage(const char* program_name)
{
	std::cout
		<< "Usage:\n  " << program_name << " [hold_ms]\n\n"
		<< "Default:\n"
		<< "  hold_ms = 250\n\n"
		<< "Controls:\n"
		<< "  w : Lift-1/front lift up\n"
		<< "  s : Lift-1/front lift down\n"
		<< "  a : Lift-2/rear lift up\n"
		<< "  d : Lift-2/rear lift down\n"
		<< "  space : stop both lifts\n"
		<< "  q : stop and quit\n\n"
		<< "Safety behavior:\n"
		<< "  Each key press holds motion only for hold_ms, then the output automatically coasts.\n"
		<< "  Hold the key or press repeatedly to keep moving.\n\n"
		<< "Direct GPIO wiring assumptions:\n"
		<< "  Lift-1/front DRV8833 (paralleled): BCM10 -> IN1+IN3, BCM9 -> IN2+IN4\n"
		<< "  Lift-2/rear DRV8833 (paralleled): BCM11 -> IN1+IN3, BCM7 -> IN2+IN4\n"
		<< "  EEP/nSLEEP must be tied high only when you are ready to drive the motors.\n";
}

bool ApplyMode(gpiod_line_request* request, LiftRig& rig, const DriveMode mode)
{
	gpiod_line_value value_a = GPIOD_LINE_VALUE_INACTIVE;
	gpiod_line_value value_b = GPIOD_LINE_VALUE_INACTIVE;

	if (mode == DriveMode::Forward)
	{
		value_a = GPIOD_LINE_VALUE_ACTIVE;
	}
	else if (mode == DriveMode::Reverse)
	{
		value_b = GPIOD_LINE_VALUE_ACTIVE;
	}

	if (gpiod_line_request_set_value(request, rig.gpio_in1_in3, value_a) < 0)
	{
		std::cerr << FormatErrno("Failed to set " + rig.name + " IN1+IN3") << '\n';
		return false;
	}

	if (gpiod_line_request_set_value(request, rig.gpio_in2_in4, value_b) < 0)
	{
		std::cerr << FormatErrno("Failed to set " + rig.name + " IN2+IN4") << '\n';
		return false;
	}

	rig.current_mode = mode;
	return true;
}

bool StopIfExpired(gpiod_line_request* request, LiftRig& rig)
{
	if (rig.current_mode == DriveMode::Coast)
	{
		return true;
	}

	if (std::chrono::steady_clock::now() < rig.command_deadline)
	{
		return true;
	}

	return ApplyMode(request, rig, DriveMode::Coast);
}

bool ReadKey(char& key)
{
	fd_set read_set;
	FD_ZERO(&read_set);
	FD_SET(STDIN_FILENO, &read_set);

	timeval timeout{};
	timeout.tv_sec = 0;
	timeout.tv_usec = 20'000;

	const int ready = select(STDIN_FILENO + 1, &read_set, nullptr, nullptr, &timeout);
	if (ready <= 0 || !FD_ISSET(STDIN_FILENO, &read_set))
	{
		return false;
	}

	return read(STDIN_FILENO, &key, 1) == 1;
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

	const int hold_ms = (argc >= 2) ? std::atoi(argv[1]) : 250;
	if (hold_ms <= 0)
	{
		std::cerr << "hold_ms must be a positive integer.\n";
		PrintUsage(argv[0]);
		return 1;
	}

	gpiod_chip* chip = gpiod_chip_open(kGpioChip);
	if (chip == nullptr)
	{
		std::cerr << FormatErrno("Failed to open /dev/gpiochip0") << '\n';
		return 1;
	}

	auto* settings = gpiod_line_settings_new();
	auto* line_config = gpiod_line_config_new();
	auto* request_config = gpiod_request_config_new();

	if (settings == nullptr || line_config == nullptr || request_config == nullptr)
	{
		std::cerr << "Failed to allocate libgpiod request objects.\n";
		gpiod_line_settings_free(settings);
		gpiod_line_config_free(line_config);
		gpiod_request_config_free(request_config);
		gpiod_chip_close(chip);
		return 1;
	}

	gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
	gpiod_line_settings_set_drive(settings, GPIOD_LINE_DRIVE_PUSH_PULL);
	gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);
	gpiod_request_config_set_consumer(request_config, "lift-keyboard-test");

	LiftRig lift1{
		"lift1/front",
		RobotConfig::MotorGPIO::LIFT_1_IN1_IN3,
		RobotConfig::MotorGPIO::LIFT_1_IN2_IN4};
	LiftRig lift2{
		"lift2/rear",
		RobotConfig::MotorGPIO::LIFT_2_IN1_IN3,
		RobotConfig::MotorGPIO::LIFT_2_IN2_IN4};

	const unsigned int offsets[4] = {
		lift1.gpio_in1_in3,
		lift1.gpio_in2_in4,
		lift2.gpio_in1_in3,
		lift2.gpio_in2_in4};
	gpiod_line_config_add_line_settings(line_config, offsets, 4, settings);
	gpiod_line_request* request = gpiod_chip_request_lines(chip, request_config, line_config);

	gpiod_line_settings_free(settings);
	gpiod_line_config_free(line_config);
	gpiod_request_config_free(request_config);

	if (request == nullptr)
	{
		std::cerr << FormatErrno("Failed to request lift GPIO lines") << '\n';
		gpiod_chip_close(chip);
		return 1;
	}

	(void)ApplyMode(request, lift1, DriveMode::Coast);
	(void)ApplyMode(request, lift2, DriveMode::Coast);

	std::cout << "Starting direct GPIO DRV8833 lift keyboard test\n";
	std::cout << "hold_ms = " << hold_ms << "\n\n";
	PrintUsage(argv[0]);

	TerminalRawMode terminal_raw_mode;
	bool running = true;

	while (running)
	{
		char key = '\0';
		if (ReadKey(key))
		{
			const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(hold_ms);
			switch (key)
			{
			case 'w':
			case 'W':
				lift1.command_deadline = deadline;
				(void)ApplyMode(request, lift1, DriveMode::Forward);
				std::cout << "Lift-1/front up\n";
				break;
			case 's':
			case 'S':
				lift1.command_deadline = deadline;
				(void)ApplyMode(request, lift1, DriveMode::Reverse);
				std::cout << "Lift-1/front down\n";
				break;
			case 'a':
			case 'A':
				lift2.command_deadline = deadline;
				(void)ApplyMode(request, lift2, DriveMode::Forward);
				std::cout << "Lift-2/rear up\n";
				break;
			case 'd':
			case 'D':
				lift2.command_deadline = deadline;
				(void)ApplyMode(request, lift2, DriveMode::Reverse);
				std::cout << "Lift-2/rear down\n";
				break;
			case ' ':
				(void)ApplyMode(request, lift1, DriveMode::Coast);
				(void)ApplyMode(request, lift2, DriveMode::Coast);
				std::cout << "Both lifts stopped\n";
				break;
			case 'q':
			case 'Q':
				running = false;
				break;
			default:
				break;
			}
		}

		if (!StopIfExpired(request, lift1) || !StopIfExpired(request, lift2))
		{
			running = false;
		}
	}

	(void)ApplyMode(request, lift1, DriveMode::Coast);
	(void)ApplyMode(request, lift2, DriveMode::Coast);
	gpiod_line_request_release(request);
	gpiod_chip_close(chip);

	std::cout << "Lift keyboard test complete. Outputs returned to coast mode.\n";
	return 0;
}
