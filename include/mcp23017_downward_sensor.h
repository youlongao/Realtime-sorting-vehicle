#ifndef MCP23017_DOWNWARD_SENSOR_H
#define MCP23017_DOWNWARD_SENSOR_H

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include "config.h"
#include "hardware_interfaces.h"
#include "mcp23017_driver.h"

namespace Robot
{
class Mcp23017DownwardSensor : public IDownwardSensor
{
public:
	Mcp23017DownwardSensor(std::shared_ptr<Mcp23017Driver> driver,
						   std::uint8_t pin,
						   bool active_on_surface = RobotConfig::Sensors::DOWNWARD_ACTIVE_ON_SURFACE);
	~Mcp23017DownwardSensor() override;

	bool start();
	void stop();

	DownwardReading latest() const override;
	bool waitForEdge(std::chrono::milliseconds timeout) override;
	void setCallback(DownwardCallback callback) override;

private:
	void workerLoop();
	void updateFromValue(bool sensor_active, bool notify_edge);

	std::shared_ptr<Mcp23017Driver> driver_;
	std::uint8_t pin_;
	bool active_on_surface_;

	mutable std::mutex mutex_;
	std::condition_variable edge_cv_;
	DownwardReading latest_reading_;
	DownwardCallback callback_;
	std::atomic_bool running_{false};
	std::thread worker_;
	bool pending_edge_{false};
	bool last_raw_value_valid_{false};
	bool last_raw_value_{false};
};
}

#endif
