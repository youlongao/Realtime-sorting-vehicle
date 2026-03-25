#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <atomic>
#include <cstdint>
#include <mutex>
#include <thread>

#include "config.h"
#include "hardware_interfaces.h"

namespace Robot
{
class ImuSensor : public IImuSensor
{
public:
	explicit ImuSensor(unsigned int bus_id = RobotConfig::I2C::BUS_ID,
					   std::uint8_t device_address = RobotConfig::IMU::ADDRESS);
	~ImuSensor() override;

	bool start();
	void stop();

	PoseData latestPose() const override;
	void setCallback(PoseCallback callback) override;

private:
	void pollingLoop();

	bool openDevice();
	void closeDevice();
	bool readPoseSample(PoseData& sample);

	unsigned int bus_id_;
	std::uint8_t device_address_;

	mutable std::mutex mutex_;
	PoseData latest_pose_;
	PoseCallback callback_;
	std::atomic_bool running_{false};
	std::thread worker_;

	int device_fd_{-1};
	float filtered_pitch_deg_{0.0F};
	float filtered_roll_deg_{0.0F};
	float integrated_yaw_deg_{0.0F};
	Timestamp last_sample_time_{SteadyClock::now()};
};
}

#endif
