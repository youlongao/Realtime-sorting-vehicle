#include "imu_sensor.h"

#include <chrono>
#include <cmath>
#include <cstring>
#include <string>
#include <thread>
#include <utility>

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
constexpr float kRadiansToDegrees = 57.2957795F;

std::string BuildI2cPath(const unsigned int bus_id)
{
	return std::string(RobotConfig::Platform::I2C_DEVICE_PREFIX) + std::to_string(bus_id);
}
}

ImuSensor::ImuSensor(const unsigned int bus_id, const std::uint8_t device_address)
	: bus_id_(bus_id), device_address_(device_address)
{
}

ImuSensor::~ImuSensor()
{
	stop();
}

bool ImuSensor::start()
{
	if (running_.load())
	{
		return true;
	}

	if (!openDevice())
	{
		return false;
	}

	running_.store(true);
	worker_ = std::thread(&ImuSensor::pollingLoop, this);
	Logger::info("IMU polling thread started.");
	return true;
}

void ImuSensor::stop()
{
	if (!running_.exchange(false))
	{
		closeDevice();
		return;
	}

	if (worker_.joinable())
	{
		worker_.join();
	}

	closeDevice();
}

PoseData ImuSensor::latestPose() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return latest_pose_;
}

void ImuSensor::setCallback(PoseCallback callback)
{
	std::lock_guard<std::mutex> lock(mutex_);
	callback_ = std::move(callback);
}

void ImuSensor::pollingLoop()
{
	while (running_.load())
	{
		PoseData sample;
		if (readPoseSample(sample))
		{
			PoseCallback callback;
			{
				std::lock_guard<std::mutex> lock(mutex_);
				latest_pose_ = sample;
				callback = callback_;
			}

			if (callback)
			{
				callback(sample);
			}
		}
		else
		{
			Logger::warn("IMU sample read failed.");
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(RobotConfig::Realtime::IMU_POLLINGS_MS));
	}
}

bool ImuSensor::openDevice()
{
	device_fd_ = open(BuildI2cPath(bus_id_).c_str(), O_RDWR);
	if (device_fd_ < 0)
	{
		Logger::error("Failed to open I2C bus for IMU: " + std::string(std::strerror(errno)));
		return false;
	}

	if (ioctl(device_fd_, I2C_SLAVE, device_address_) < 0)
	{
		Logger::error("Failed to select IMU address: " + std::string(std::strerror(errno)));
		closeDevice();
		return false;
	}

	std::uint8_t wake_command[2] = {RobotConfig::IMU::POWER_MGMT_REG, 0x00};
	if (write(device_fd_, wake_command, sizeof(wake_command)) != static_cast<ssize_t>(sizeof(wake_command)))
	{
		Logger::error("Failed to wake IMU: " + std::string(std::strerror(errno)));
		closeDevice();
		return false;
	}

	last_sample_time_ = SteadyClock::now();
	return true;
}

void ImuSensor::closeDevice()
{
	if (device_fd_ >= 0)
	{
		close(device_fd_);
		device_fd_ = -1;
	}
}

bool ImuSensor::readPoseSample(PoseData& sample)
{
	if (device_fd_ < 0)
	{
		return false;
	}

	const std::uint8_t start_register = RobotConfig::IMU::ACCEL_START_REG;
	if (write(device_fd_, &start_register, 1) != 1)
	{
		return false;
	}

	std::uint8_t raw_bytes[14] = {};
	if (read(device_fd_, raw_bytes, sizeof(raw_bytes)) != static_cast<ssize_t>(sizeof(raw_bytes)))
	{
		return false;
	}

	auto read_word = [&raw_bytes](const int index) -> std::int16_t {
		return static_cast<std::int16_t>((raw_bytes[index] << 8) | raw_bytes[index + 1]);
	};

	const float accel_x = static_cast<float>(read_word(0)) / RobotConfig::IMU::ACCEL_SCALE;
	const float accel_y = static_cast<float>(read_word(2)) / RobotConfig::IMU::ACCEL_SCALE;
	const float accel_z = static_cast<float>(read_word(4)) / RobotConfig::IMU::ACCEL_SCALE;
	const float gyro_x = static_cast<float>(read_word(8)) / RobotConfig::IMU::GYRO_SCALE;
	const float gyro_y = static_cast<float>(read_word(10)) / RobotConfig::IMU::GYRO_SCALE;
	const float gyro_z = static_cast<float>(read_word(12)) / RobotConfig::IMU::GYRO_SCALE;

	const auto now = SteadyClock::now();
	const float dt =
		std::chrono::duration_cast<std::chrono::duration<float>>(now - last_sample_time_).count();
	last_sample_time_ = now;

	const float accel_pitch =
		std::atan2(-accel_x, std::sqrt((accel_y * accel_y) + (accel_z * accel_z))) * kRadiansToDegrees;
	const float accel_roll = std::atan2(accel_y, accel_z) * kRadiansToDegrees;

	const auto previous_pose = latestPose();

	if (!previous_pose.valid)
	{
		filtered_pitch_deg_ = accel_pitch;
		filtered_roll_deg_ = accel_roll;
	}

	filtered_pitch_deg_ = lowPassFilter(
		filtered_pitch_deg_ + (gyro_y * dt),
		accel_pitch,
		RobotConfig::IMU::COMPLEMENTARY_ALPHA);
	filtered_roll_deg_ = lowPassFilter(
		filtered_roll_deg_ + (gyro_x * dt),
		accel_roll,
		RobotConfig::IMU::COMPLEMENTARY_ALPHA);
	integrated_yaw_deg_ += gyro_z * dt;

	sample.pitch_deg = filtered_pitch_deg_;
	sample.roll_deg = filtered_roll_deg_;
	sample.yaw_deg = integrated_yaw_deg_;
	sample.timestamp = now;
	sample.valid = true;
	return true;
}
}
