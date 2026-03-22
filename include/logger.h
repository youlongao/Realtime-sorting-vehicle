#ifndef LOGGER_H
#define LOGGER_H

#include <string>

namespace Robot
{
enum class LogLevel
{
	Debug,
	Info,
	Warning,
	Error
};

class Logger
{
public:
	static void debug(const std::string& message);
	static void info(const std::string& message);
	static void warn(const std::string& message);
	static void error(const std::string& message);

private:
	static void log(LogLevel level, const std::string& message);
};
}

#endif
