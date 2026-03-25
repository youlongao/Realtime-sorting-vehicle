#ifndef DOWNWARD_SENSOR_H
#define DOWNWARD_SENSOR_H

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

#include "config.h"
#include "hardware_interfaces.h"

struct gpiod_chip;
struct gpiod_line_request;
struct gpiod_edge_event_buffer;

namespace Robot
{
class DownwardSensor : public IDownwardSensor
{
public:
	explicit DownwardSensor(std::string chip_path = RobotConfig::Platform::GPIO_CHIP,
							 unsigned int line_offset = RobotConfig::GPIO::DOWNWARD_DETECTOR,
							 bool active_on_surface = RobotConfig::Sensors::DOWNWARD_ACTIVE_ON_SURFACE);
	~DownwardSensor() override;

	bool start();
	void stop();

	DownwardReading latest() const override;
	bool waitForEdge(std::chrono::milliseconds timeout) override;
	void setCallback(DownwardCallback callback) override;

private:
	void workerLoop();
	void updateFromValue(bool sensor_active);
	bool initialiseRequest();
	void releaseRequest();

	std::string chip_path_;
	unsigned int line_offset_;
	bool active_on_surface_;

	mutable std::mutex mutex_;
	std::condition_variable edge_cv_;
	DownwardReading latest_reading_;
	DownwardCallback callback_;
	std::atomic_bool running_{false};
	std::thread worker_;
	bool pending_edge_{false};

	gpiod_chip* chip_{nullptr};
	gpiod_line_request* request_{nullptr};
	gpiod_edge_event_buffer* event_buffer_{nullptr};
};
}

#endif
