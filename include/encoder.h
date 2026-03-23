#ifndef ENCODER_H
#define ENCODER_H

#include <atomic>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "config.h"
#include "hardware_interfaces.h"

struct gpiod_chip;
struct gpiod_edge_event_buffer;
struct gpiod_line_request;

namespace Robot
{
class Encoder : public IEncoderFeedback
{
public:
	explicit Encoder(unsigned int channel_a_offset,
					 std::optional<unsigned int> channel_b_offset = std::nullopt,
					 float meters_per_tick = RobotConfig::Encoder::DEFAULT_METERS_PER_TICK,
					 std::string chip_path = RobotConfig::Platform::GPIO_CHIP);
	~Encoder() override;

	bool start();
	void stop();

	std::int64_t readTicks() const override;
	float getSpeed() const override;
	float getDistance() const override;
	EncoderSample latestSample() const override;
	void reset() override;
	void setCallback(EncoderCallback callback);

private:
	void workerLoop();
	void updateFromState(bool channel_a_active, bool channel_b_active, Timestamp timestamp);

#if defined(__linux__) && defined(CLIMBING_ROBOT_HAS_LIBGPIOD)
	bool initialiseRequest();
	void releaseRequest();
#endif

	unsigned int channel_a_offset_;
	std::optional<unsigned int> channel_b_offset_;
	float meters_per_tick_;
	std::string chip_path_;

	mutable std::mutex mutex_;
	EncoderSample sample_;
	EncoderCallback callback_;
	std::atomic_bool running_{false};
	std::thread worker_;
	std::uint8_t previous_state_{0U};

#if defined(__linux__) && defined(CLIMBING_ROBOT_HAS_LIBGPIOD)
	gpiod_chip* chip_{nullptr};
	gpiod_line_request* request_{nullptr};
	gpiod_edge_event_buffer* event_buffer_{nullptr};
#endif
};
}

#endif
