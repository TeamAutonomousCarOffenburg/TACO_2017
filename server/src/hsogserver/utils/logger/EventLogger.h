#pragma once

#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

#include <boost/shared_ptr.hpp>

#include "utils/logger/IEventLogger.h"

class EventLogger : public virtual taco::IEventLogger
{
  public:
	static EventLogger *getLogger(std::string filename)
	{
		if (logger)
			return logger;

		logger = new EventLogger(filename);
		return logger;
	}

	void logDecodeEvent(uint32_t timestamp, std::string name, double value) const;
	void logPose(const std::string &name, const taco::Pose2D &pose) const;
	void logTimeTaken(const std::string &name, long duration) const;

	void start() const;
	void stop() const;

	~EventLogger()
	{
		logger = nullptr;
		stop();
	}

  protected:
	EventLogger(std::string filename)
	{
		_filename = filename;
	}

  private:
	void writeToLog(const std::string &toWrite) const;

	static EventLogger *logger;

	std::string _filename;
	const char *del = ";";

	mutable bool _started = false;
	mutable std::ofstream _log;
	mutable std::mutex writeLock;
	EventLogger(const EventLogger &);
};
