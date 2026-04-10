#ifndef MCP23017_LIMIT_SWITCH_H
#define MCP23017_LIMIT_SWITCH_H

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include "config.h"
#include "hardware_interfaces.h"
#include "limit_switch.h"
#include "mcp23017_driver.h"

namespace Robot
{
class Mcp23017LimitSwitch : public ILimitSwitch
{
public:
	Mcp23017LimitSwitch(std::shared_ptr<Mcp23017Driver> driver,
						std::uint8_t pin,
						LimitRole role = LimitRole::Generic,
						bool active_low = RobotConfig::Limits::ACTIVE_LOW);
	~Mcp23017LimitSwitch() override;

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
	void updateFromValue(bool raw_value, bool notify_edge);

	std::shared_ptr<Mcp23017Driver> driver_;
	std::uint8_t pin_;
	LimitRole role_;
	bool active_low_;

	mutable std::mutex mutex_;
	std::condition_variable trigger_cv_;
	LimitSwitchState state_;
	LimitSwitchCallback callback_;
	std::atomic_bool running_{false};
	std::thread worker_;
	bool pending_trigger_{false};
	bool last_raw_value_valid_{false};
	bool last_raw_value_{false};
};
}

#endif
