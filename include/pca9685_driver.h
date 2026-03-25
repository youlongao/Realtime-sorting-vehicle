#ifndef PCA9685_DRIVER_H
#define PCA9685_DRIVER_H

#include <cstdint>
#include <mutex>

#include "config.h"

namespace Robot
{
class Pca9685Driver
{
public:
	explicit Pca9685Driver(unsigned int bus_id = RobotConfig::I2C::BUS_ID,
						   std::uint8_t device_address = RobotConfig::I2C::PCA9685_ADDR,
						   int pwm_frequency_hz = RobotConfig::I2C::PWM_FREQ);
	~Pca9685Driver();

	bool start();
	void stop();
	bool isReady() const;

	bool setDutyCycle(std::uint8_t channel, float normalized_duty);
	bool setPulseWidthUs(std::uint8_t channel, float pulse_width_us);
	bool disableChannel(std::uint8_t channel);

private:
	bool setChannelRaw(std::uint8_t channel, std::uint16_t on_count, std::uint16_t off_count);

	bool openDevice();
	void closeDevice();
	bool configureFrequency();
	bool writeRegister(std::uint8_t reg, std::uint8_t value);
	bool readRegister(std::uint8_t reg, std::uint8_t& value);

	mutable std::mutex mutex_;
	unsigned int bus_id_;
	std::uint8_t device_address_;
	int pwm_frequency_hz_;
	bool ready_{false};

	int device_fd_{-1};
};
}

#endif
