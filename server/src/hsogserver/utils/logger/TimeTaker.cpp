#include "TimeTaker.h"

using namespace std::chrono;

TimeTaker::TimeTaker(const std::string &name)
{
	auto timeNow = system_clock::now();
	startTime = duration_cast<milliseconds>(timeNow.time_since_epoch()).count();
	this->name = name;
}

TimeTaker::~TimeTaker()
{
	auto timeNow = system_clock::now();
	long long endTime = duration_cast<milliseconds>(timeNow.time_since_epoch()).count();
	EventLogger::getLogger("")->logTimeTaken(name, endTime - startTime);
}
