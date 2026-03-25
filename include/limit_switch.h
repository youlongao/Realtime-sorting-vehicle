#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

#include "config.h"
#include "hardware_interfaces.h"

struct gpiod_chip;
struct gpiod_edge_event_buffer;
struct gpiod_line_request;

namespace Robot
{
enum class LimitRole
{
	Generic,
	Upper,
	Lower
};

class LimitSwitch : public ILimitSwitch
{
public:
	explicit LimitSwitch(unsigned int line_offset,
						 LimitRole role = LimitRole::Generic,
						 bool active_low = false,
						 std::string chip_path = RobotConfig::Platform::GPIO_CHIP);
	~LimitSwitch() override;

	bool start();
	void stop();

	bool isTriggered() const;
	bool isUpperLimit() const;
	bool isLowerLimit() const;

	LimitSwitchState latestState() const override;
	bool waitForTrigger(std::chrono::milliseconds timeout) override;
	void setCallback(LimitSwitchCallback callback) override;

private:
	void workerLoop();
	void updateFromValue(bool line_active);
	bool initialiseRequest();
	void releaseRequest();

	unsigned int line_offset_;
	LimitRole role_;
	bool active_low_;
	std::string chip_path_;

	mutable std::mutex mutex_;
	std::condition_variable trigger_cv_;
	LimitSwitchState state_;
	LimitSwitchCallback callback_;
	std::atomic_bool running_{false};
	std::thread worker_;
	bool pending_trigger_{false};

	gpiod_chip* chip_{nullptr};
	gpiod_line_request* request_{nullptr};
	gpiod_edge_event_buffer* event_buffer_{nullptr};
};
}

#endif
