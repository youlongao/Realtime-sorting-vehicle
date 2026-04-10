#ifndef MCP23017_DRIVER_H
#define MCP23017_DRIVER_H

#include <cstdint>
#include <mutex>

#include "config.h"

namespace Robot
{
class Mcp23017Driver
{
public:
	explicit Mcp23017Driver(unsigned int bus_id = RobotConfig::I2C::BUS_ID,
							std::uint8_t device_address = RobotConfig::I2C::MCP23017_ADDR);
	~Mcp23017Driver();

	bool start();
	void stop();
	bool isReady() const;

	bool configureInput(std::uint8_t pin, bool pullup_enabled);
	bool readPin(std::uint8_t pin, bool& value);

private:
	bool openDeviceLocked();
	void closeDeviceLocked();
	bool writeRegisterLocked(std::uint8_t reg, std::uint8_t value);
	bool readRegisterLocked(std::uint8_t reg, std::uint8_t& value);
	bool writeBitLocked(std::uint8_t reg, std::uint8_t bit, bool value, std::uint8_t& cache);

	mutable std::mutex mutex_;
	unsigned int bus_id_;
	std::uint8_t device_address_;
	bool ready_{false};
	int device_fd_{-1};
	std::uint8_t iodira_{0xFF};
	std::uint8_t iodirb_{0xFF};
	std::uint8_t gppua_{0x00};
	std::uint8_t gppub_{0x00};
};
}

#endif
