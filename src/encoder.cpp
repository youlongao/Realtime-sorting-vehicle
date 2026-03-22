#include "encoder.h"

#include <array>
#include <chrono>
#include <cstring>
#include <utility>

#include "logger.h"

#if defined(__linux__) && defined(CLIMBING_ROBOT_HAS_LIBGPIOD)
#include <cerrno>

#include <gpiod.h>
#endif

namespace Robot
{
namespace
{
std::string FormatErrno(const std::string& prefix)
{
#if defined(__linux__) && defined(CLIMBING_ROBOT_HAS_LIBGPIOD)
	return prefix + ": " + std::strerror(errno);
#else
	return prefix;
#endif
}
}

Encoder::Encoder(const unsigned int channel_a_offset,
				 std::optional<unsigned int> channel_b_offset,
				 const float meters_per_tick,
				 std::string chip_path)
	: channel_a_offset_(channel_a_offset),
	  channel_b_offset_(channel_b_offset),
	  meters_per_tick_(meters_per_tick),
	  chip_path_(std::move(chip_path))
{
}

Encoder::~Encoder()
{
	stop();
}

bool Encoder::start()
{
	if (running_.load())
	{
		return true;
	}

#if defined(__linux__) && defined(CLIMBING_ROBOT_HAS_LIBGPIOD)
	if (!initialiseRequest())
	{
		return false;
	}

	running_.store(true);
	worker_ = std::thread(&Encoder::workerLoop, this);
	return true;
#else
	Logger::warn("Encoder start skipped because libgpiod v2 is unavailable.");
	return false;
#endif
}

void Encoder::stop()
{
	if (!running_.exchange(false))
	{
#if defined(__linux__) && defined(CLIMBING_ROBOT_HAS_LIBGPIOD)
		releaseRequest();
#endif
		return;
	}

	if (worker_.joinable())
	{
		worker_.join();
	}

#if defined(__linux__) && defined(CLIMBING_ROBOT_HAS_LIBGPIOD)
	releaseRequest();
#endif
}

std::int64_t Encoder::readTicks() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return sample_.ticks;
}

float Encoder::getSpeed() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return sample_.speed_mps;
}

float Encoder::getDistance() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return sample_.distance_m;
}

EncoderSample Encoder::latestSample() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return sample_;
}

void Encoder::reset()
{
	std::lock_guard<std::mutex> lock(mutex_);
	sample_ = {};
}

void Encoder::setCallback(EncoderCallback callback)
{
	std::lock_guard<std::mutex> lock(mutex_);
	callback_ = std::move(callback);
}

void Encoder::workerLoop()
{
#if defined(__linux__) && defined(CLIMBING_ROBOT_HAS_LIBGPIOD)
	while (running_.load())
	{
		if (request_ == nullptr)
		{
			break;
		}

		const int wait_result = gpiod_line_request_wait_edge_events(
			request_,
			std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::milliseconds(100)).count());

		if (!running_.load())
		{
			break;
		}

		if (wait_result < 0)
		{
			Logger::error(FormatErrno("Encoder wait failed"));
			break;
		}

		if (wait_result == 0)
		{
			continue;
		}

		const int events_read = gpiod_line_request_read_edge_events(
			request_,
			event_buffer_,
			RobotConfig::Encoder::EVENT_BUFFER_SIZE);

		if (events_read < 0)
		{
			Logger::error(FormatErrno("Encoder edge read failed"));
			continue;
		}

		for (int index = 0; index < events_read; ++index)
		{
			const auto* event = gpiod_edge_event_buffer_get_event(event_buffer_, index);
			if (event == nullptr)
			{
				continue;
			}

			const bool a_active =
				gpiod_line_request_get_value(request_, channel_a_offset_) == GPIOD_LINE_VALUE_ACTIVE;
			bool b_active = false;
			if (channel_b_offset_.has_value())
			{
				b_active =
					gpiod_line_request_get_value(request_, *channel_b_offset_) == GPIOD_LINE_VALUE_ACTIVE;
			}

			updateFromState(a_active, b_active, SteadyClock::now());
		}
	}
#endif
}

void Encoder::updateFromState(const bool channel_a_active,
							  const bool channel_b_active,
							  const Timestamp timestamp)
{
	static constexpr std::array<std::array<int, 4>, 4> transition_table{{
		{{0, -1, 1, 0}},
		{{1, 0, 0, -1}},
		{{-1, 0, 0, 1}},
		{{0, 1, -1, 0}},
	}};

	EncoderCallback callback;
	EncoderSample snapshot;

	{
		std::lock_guard<std::mutex> lock(mutex_);
		const std::uint8_t new_state =
			static_cast<std::uint8_t>((channel_a_active ? 0x02U : 0x00U) | (channel_b_active ? 0x01U : 0x00U));

		int delta_ticks = 1;
		if (channel_b_offset_.has_value())
		{
			delta_ticks = transition_table[previous_state_][new_state];
		}

		const float previous_distance = sample_.distance_m;
		const Timestamp previous_timestamp = sample_.timestamp;

		sample_.ticks += delta_ticks;
		sample_.distance_m = static_cast<float>(sample_.ticks) * meters_per_tick_;
		sample_.timestamp = timestamp;
		sample_.valid = true;

		const float dt =
			std::chrono::duration_cast<std::chrono::duration<float>>(timestamp - previous_timestamp).count();
		if (dt > 0.0F)
		{
			sample_.speed_mps = (sample_.distance_m - previous_distance) / dt;
		}

		previous_state_ = new_state;
		callback = callback_;
		snapshot = sample_;
	}

	if (callback)
	{
		callback(snapshot);
	}
}

#if defined(__linux__) && defined(CLIMBING_ROBOT_HAS_LIBGPIOD)
bool Encoder::initialiseRequest()
{
	chip_ = gpiod_chip_open(chip_path_.c_str());
	if (chip_ == nullptr)
	{
		Logger::error(FormatErrno("Failed to open gpiochip for encoder"));
		return false;
	}

	auto* settings = gpiod_line_settings_new();
	auto* config = gpiod_line_config_new();
	auto* request_config = gpiod_request_config_new();

	if (settings == nullptr || config == nullptr || request_config == nullptr)
	{
		Logger::error("Failed to allocate libgpiod objects for encoder.");
		gpiod_line_settings_free(settings);
		gpiod_line_config_free(config);
		gpiod_request_config_free(request_config);
		releaseRequest();
		return false;
	}

	gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
	gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_BOTH);
	gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_DISABLED);
	gpiod_line_settings_set_event_clock(settings, GPIOD_LINE_CLOCK_MONOTONIC);

	unsigned int offsets[2] = {channel_a_offset_, channel_b_offset_.value_or(channel_a_offset_)};
	const std::size_t offset_count = channel_b_offset_.has_value() ? 2U : 1U;
	gpiod_line_config_add_line_settings(config, offsets, offset_count, settings);
	gpiod_request_config_set_consumer(request_config, "encoder");
	gpiod_request_config_set_event_buffer_size(request_config, RobotConfig::Encoder::EVENT_BUFFER_SIZE);

	request_ = gpiod_chip_request_lines(chip_, request_config, config);
	event_buffer_ = gpiod_edge_event_buffer_new(RobotConfig::Encoder::EVENT_BUFFER_SIZE);

	gpiod_line_settings_free(settings);
	gpiod_line_config_free(config);
	gpiod_request_config_free(request_config);

	if (request_ == nullptr || event_buffer_ == nullptr)
	{
		Logger::error(FormatErrno("Failed to request encoder GPIO line(s)"));
		releaseRequest();
		return false;
	}

	const bool a_active = gpiod_line_request_get_value(request_, channel_a_offset_) == GPIOD_LINE_VALUE_ACTIVE;
	const bool b_active = channel_b_offset_.has_value() &&
						  gpiod_line_request_get_value(request_, *channel_b_offset_) == GPIOD_LINE_VALUE_ACTIVE;
	previous_state_ =
		static_cast<std::uint8_t>((a_active ? 0x02U : 0x00U) | (b_active ? 0x01U : 0x00U));
	return true;
}

void Encoder::releaseRequest()
{
	if (event_buffer_ != nullptr)
	{
		gpiod_edge_event_buffer_free(event_buffer_);
		event_buffer_ = nullptr;
	}

	if (request_ != nullptr)
	{
		gpiod_line_request_release(request_);
		request_ = nullptr;
	}

	if (chip_ != nullptr)
	{
		gpiod_chip_close(chip_);
		chip_ = nullptr;
	}
}
#endif
}
