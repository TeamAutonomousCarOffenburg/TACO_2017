#pragma once

#include "EventLogger.h"

class TimeTaker
{
  public:
	TimeTaker(const std::string &name);
	~TimeTaker();

  private:
	long long startTime = 0;
	std::string name;
};
