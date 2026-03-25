#ifndef FRONT_DISTANCE_SENSOR_H
#define FRONT_DISTANCE_SENSOR_H

#include <mutex>
#include <string>

#include "config.h"
#include "hardware_interfaces.h"

struct gpiod_chip;
struct gpiod_edge_event_buffer;
struct gpiod_line_request;

namespace Robot
{
class FrontDistanceSensor : public IFrontDistanceSensor
{
public:
	explicit FrontDistanceSensor(std::string chip_path = RobotConfig::Platform::GPIO_CHIP,
								 unsigned int trig_offset = RobotConfig::GPIO::ULTRASONIC_TRIG,
								 unsigned int echo_offset = RobotConfig::GPIO::ULTRASONIC_ECHO);
	~FrontDistanceSensor() override;

	bool start();
	void stop();

	DistanceReading readBlocking(std::chrono::microseconds timeout) override;
	DistanceReading latest() const override;
	void setCallback(DistanceCallback callback) override;

private:
	float pulseWidthToDistance(std::uint64_t pulse_width_ns) const;
	bool initialiseRequests();
	void releaseRequests();
	void clearPendingEchoEvents();

	std::string chip_path_;
	unsigned int trig_offset_;
	unsigned int echo_offset_;

	mutable std::mutex mutex_;
	DistanceReading latest_reading_;
	DistanceCallback callback_;

	gpiod_chip* chip_{nullptr};
	gpiod_line_request* trig_request_{nullptr};
	gpiod_line_request* echo_request_{nullptr};
	gpiod_edge_event_buffer* event_buffer_{nullptr};
};
}

#endif
