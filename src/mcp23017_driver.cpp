#include "mcp23017_driver.h"

#include <cstring>
#include <string>

#include "logger.h"

#include <cerrno>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace Robot
{
namespace
{
constexpr std::uint8_t kIodirARegister = 0x00;
constexpr std::uint8_t kIodirBRegister = 0x01;
constexpr std::uint8_t kGppuARegister = 0x0C;
constexpr std::uint8_t kGppuBRegister = 0x0D;
constexpr std::uint8_t kGpioARegister = 0x12;
constexpr std::uint8_t kGpioBRegister = 0x13;

std::string BuildI2cPath(const unsigned int bus_id)
{
	return std::string(RobotConfig::Platform::I2C_DEVICE_PREFIX) + std::to_string(bus_id);
}
}

Mcp23017Driver::Mcp23017Driver(const unsigned int bus_id, const std::uint8_t device_address)
	: bus_id_(bus_id),
	  device_address_(device_address)
{
}

Mcp23017Driver::~Mcp23017Driver()
{
	stop();
}

bool Mcp23017Driver::start()
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (ready_)
	{
		return true;
	}

	if (!openDeviceLocked())
	{
		return false;
	}

	ready_ =
		writeRegisterLocked(kIodirARegister, iodira_) &&
		writeRegisterLocked(kIodirBRegister, iodirb_) &&
		writeRegisterLocked(kGppuARegister, gppua_) &&
		writeRegisterLocked(kGppuBRegister, gppub_);

	if (!ready_)
	{
		closeDeviceLocked();
	}

	return ready_;
}

void Mcp23017Driver::stop()
{
	std::lock_guard<std::mutex> lock(mutex_);
	ready_ = false;
	closeDeviceLocked();
}

bool Mcp23017Driver::isReady() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return ready_;
}

bool Mcp23017Driver::configureInput(const std::uint8_t pin, const bool pullup_enabled)
{
	if (pin >= 16U || !start())
	{
		return false;
	}

	std::lock_guard<std::mutex> lock(mutex_);

	if (pin < 8U)
	{
		return writeBitLocked(kIodirARegister, pin, true, iodira_) &&
			   writeBitLocked(kGppuARegister, pin, pullup_enabled, gppua_);
	}

	const std::uint8_t bit = static_cast<std::uint8_t>(pin - 8U);
	return writeBitLocked(kIodirBRegister, bit, true, iodirb_) &&
		   writeBitLocked(kGppuBRegister, bit, pullup_enabled, gppub_);
}

bool Mcp23017Driver::readPin(const std::uint8_t pin, bool& value)
{
	if (pin >= 16U || !start())
	{
		return false;
	}

	std::lock_guard<std::mutex> lock(mutex_);
	std::uint8_t gpio_value = 0U;

	if (pin < 8U)
	{
		if (!readRegisterLocked(kGpioARegister, gpio_value))
		{
			return false;
		}

		value = (gpio_value & static_cast<std::uint8_t>(1U << pin)) != 0U;
		return true;
	}

	const std::uint8_t bit = static_cast<std::uint8_t>(pin - 8U);
	if (!readRegisterLocked(kGpioBRegister, gpio_value))
	{
		return false;
	}

	value = (gpio_value & static_cast<std::uint8_t>(1U << bit)) != 0U;
	return true;
}

bool Mcp23017Driver::openDeviceLocked()
{
	device_fd_ = open(BuildI2cPath(bus_id_).c_str(), O_RDWR);
	if (device_fd_ < 0)
	{
		Logger::error("Failed to open I2C bus for MCP23017: " + std::string(std::strerror(errno)));
		return false;
	}

	if (ioctl(device_fd_, I2C_SLAVE, device_address_) < 0)
	{
		Logger::error("Failed to select MCP23017 I2C address: " + std::string(std::strerror(errno)));
		closeDeviceLocked();
		return false;
	}

	return true;
}

void Mcp23017Driver::closeDeviceLocked()
{
	if (device_fd_ >= 0)
	{
		close(device_fd_);
		device_fd_ = -1;
	}
}

bool Mcp23017Driver::writeRegisterLocked(const std::uint8_t reg, const std::uint8_t value)
{
	const std::uint8_t payload[2] = {reg, value};
	return write(device_fd_, payload, sizeof(payload)) == static_cast<ssize_t>(sizeof(payload));
}

bool Mcp23017Driver::readRegisterLocked(const std::uint8_t reg, std::uint8_t& value)
{
	if (write(device_fd_, &reg, 1) != 1)
	{
		return false;
	}

	return read(device_fd_, &value, 1) == 1;
}

bool Mcp23017Driver::writeBitLocked(const std::uint8_t reg,
									const std::uint8_t bit,
									const bool value,
									std::uint8_t& cache)
{
	const std::uint8_t mask = static_cast<std::uint8_t>(1U << bit);
	if (value)
	{
		cache = static_cast<std::uint8_t>(cache | mask);
	}
	else
	{
		cache = static_cast<std::uint8_t>(cache & ~mask);
	}

	return writeRegisterLocked(reg, cache);
}
}
