#pragma once

#include <chrono>
#include <ctime>

#include "general/ComponentFactory.h"
#include "utils/logger/EventLogger.h"

namespace taco
{
class LogComponentFactory : public ComponentFactory
{
  public:
	LogComponentFactory()
	{
		_baseLogfile = generateFileName();
	}

	IEventLogger::Ptr createEventLogger() const override
	{
		return IEventLogger::Ptr(EventLogger::getLogger(_baseLogfile));
	}

  private:
	std::string _baseLogfile;

	const std::string generateFileName()
	{
		std::stringstream ss;
		ss << "taco_log_" << getTimeString() << ".csv";
		return ss.str();
	}

	const std::string getTimeString()
	{
		char buffer[25];
		struct tm *tm_info;

		time_t timer;
		time(&timer);
		tm_info = localtime(&timer);

		strftime(buffer, 25, "%Y_%m_%d_%H_%M_%S", tm_info);
		std::string timeString = std::string(buffer);
		return timeString;
	}
};
}
