#include "logger.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <mutex>

namespace Robot
{
namespace
{
std::mutex g_log_mutex;

const char* ToString(const LogLevel level)
{
	switch (level)
	{
	case LogLevel::Debug:
		return "DEBUG";
	case LogLevel::Info:
		return "INFO";
	case LogLevel::Warning:
		return "WARN";
	case LogLevel::Error:
		return "ERROR";
	default:
		return "LOG";
	}
}
}

void Logger::debug(const std::string& message)
{
	log(LogLevel::Debug, message);
}

void Logger::info(const std::string& message)
{
	log(LogLevel::Info, message);
}

void Logger::warn(const std::string& message)
{
	log(LogLevel::Warning, message);
}

void Logger::error(const std::string& message)
{
	log(LogLevel::Error, message);
}

void Logger::log(const LogLevel level, const std::string& message)
{
	const auto now = std::chrono::system_clock::now();
	const std::time_t raw_time = std::chrono::system_clock::to_time_t(now);

	std::tm time_info{};
#if defined(_WIN32)
	localtime_s(&time_info, &raw_time);
#else
	localtime_r(&raw_time, &time_info);
#endif

	std::lock_guard<std::mutex> lock(g_log_mutex);
	std::cout << '[' << std::put_time(&time_info, "%H:%M:%S")
			  << "] [" << ToString(level) << "] "
			  << message << '\n';
}
}
