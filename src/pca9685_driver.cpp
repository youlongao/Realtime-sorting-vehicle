#include "pca9685_driver.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>

#include "logger.h"
#include "utils.h"

#include <cerrno>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace Robot
{
namespace
{
constexpr std::uint8_t kMode1Register = 0x00;
constexpr std::uint8_t kMode2Register = 0x01;
constexpr std::uint8_t kLed0Register = 0x06;
constexpr std::uint8_t kPrescaleRegister = 0xFE;
constexpr std::uint8_t kAutoIncrement = 0x20;
constexpr std::uint8_t kSleepBit = 0x10;
constexpr std::uint8_t kRestartBit = 0x80;
constexpr std::uint8_t kOutDrvBit = 0x04;
}

Pca9685Driver::Pca9685Driver(const unsigned int bus_id,
							 const std::uint8_t device_address,
							 const int pwm_frequency_hz)
	: bus_id_(bus_id),
	  device_address_(device_address),
	  pwm_frequency_hz_(pwm_frequency_hz)
{
}

Pca9685Driver::~Pca9685Driver()
{
	stop();
}

bool Pca9685Driver::start()
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (ready_)
	{
		return true;
	}

	if (!openDevice())
	{
		return false;
	}

	ready_ = configureFrequency();
	return ready_;
}

void Pca9685Driver::stop()
{
	std::lock_guard<std::mutex> lock(mutex_);
	ready_ = false;

	closeDevice();
}

bool Pca9685Driver::isReady() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return ready_;
}

bool Pca9685Driver::setDutyCycle(const std::uint8_t channel, const float normalized_duty)
{
	if (!start())
	{
		return false;
	}

	const float duty = clamp(normalized_duty, 0.0F, 1.0F);
	const auto off_count = static_cast<std::uint16_t>(
		std::lround(duty * (RobotConfig::PWM::PCA9685_RESOLUTION - 1.0F)));
	return setChannelRaw(channel, 0U, off_count);
}

bool Pca9685Driver::setPulseWidthUs(const std::uint8_t channel, const float pulse_width_us)
{
	if (!start())
	{
		return false;
	}

	const float period_us = 1'000'000.0F / static_cast<float>(pwm_frequency_hz_);
	const float normalized_duty = clamp(pulse_width_us / period_us, 0.0F, 1.0F);
	return setDutyCycle(channel, normalized_duty);
}

bool Pca9685Driver::disableChannel(const std::uint8_t channel)
{
	if (!start())
	{
		return false;
	}

	return setChannelRaw(channel, 0U, 0U);
}

bool Pca9685Driver::setChannelRaw(const std::uint8_t channel,
								  const std::uint16_t on_count,
								  const std::uint16_t off_count)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (!ready_ || device_fd_ < 0 || channel >= 16)
	{
		return false;
	}

	const std::uint8_t register_address = static_cast<std::uint8_t>(kLed0Register + (4 * channel));
	const std::array<std::uint8_t, 5> payload{
		register_address,
		static_cast<std::uint8_t>(on_count & 0xFFU),
		static_cast<std::uint8_t>((on_count >> 8U) & 0x0FU),
		static_cast<std::uint8_t>(off_count & 0xFFU),
		static_cast<std::uint8_t>((off_count >> 8U) & 0x0FU)};
	return write(device_fd_, payload.data(), payload.size()) == static_cast<ssize_t>(payload.size());
}

bool Pca9685Driver::openDevice()
{
	device_fd_ = open((std::string(RobotConfig::Platform::I2C_DEVICE_PREFIX) + std::to_string(bus_id_)).c_str(), O_RDWR);
	if (device_fd_ < 0)
	{
		Logger::error("Failed to open I2C bus for PCA9685: " + std::string(std::strerror(errno)));
		return false;
	}

	if (ioctl(device_fd_, I2C_SLAVE, device_address_) < 0)
	{
		Logger::error("Failed to select PCA9685 I2C address: " + std::string(std::strerror(errno)));
		closeDevice();
		return false;
	}

	return true;
}

void Pca9685Driver::closeDevice()
{
	if (device_fd_ >= 0)
	{
		close(device_fd_);
		device_fd_ = -1;
	}
}

bool Pca9685Driver::configureFrequency()
{
	std::uint8_t current_mode = 0U;
	if (!readRegister(kMode1Register, current_mode))
	{
		return false;
	}

	// Force MODE2 into totem-pole output mode so DRV8833 inputs see a
	// strong logic high instead of the weaker default open-drain behavior.
	if (!writeRegister(kMode2Register, kOutDrvBit))
	{
		return false;
	}

	const auto prescale = static_cast<std::uint8_t>(std::lround(
		(RobotConfig::PWM::PCA9685_OSCILLATOR_HZ /
		 (RobotConfig::PWM::PCA9685_RESOLUTION * static_cast<float>(pwm_frequency_hz_))) -
		1.0F));

	if (!writeRegister(kMode1Register, static_cast<std::uint8_t>((current_mode & ~kRestartBit) | kSleepBit)))
	{
		return false;
	}

	if (!writeRegister(kPrescaleRegister, prescale))
	{
		return false;
	}

	if (!writeRegister(kMode1Register, static_cast<std::uint8_t>(current_mode | kAutoIncrement)))
	{
		return false;
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(1));
	return writeRegister(kMode1Register, static_cast<std::uint8_t>(current_mode | kAutoIncrement | kRestartBit));
}

bool Pca9685Driver::writeRegister(const std::uint8_t reg, const std::uint8_t value)
{
	const std::array<std::uint8_t, 2> payload{reg, value};
	return write(device_fd_, payload.data(), payload.size()) == static_cast<ssize_t>(payload.size());
}

bool Pca9685Driver::readRegister(const std::uint8_t reg, std::uint8_t& value)
{
	if (write(device_fd_, &reg, 1) != 1)
	{
		return false;
	}

	return read(device_fd_, &value, 1) == 1;
}
}
