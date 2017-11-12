#include "EventLogger.h"

using namespace std;
using namespace taco;

void EventLogger::logDecodeEvent(uint32_t timestamp, string name, double value) const
{
	stringstream ss;
	ss << "DECODE" << del << name << del << timestamp << del << value;

	writeToLog(ss.str());
}

void EventLogger::logPose(const string &name, const Pose2D &pose) const
{
	stringstream ss;
	ss << "POSE" << del << name << del << pose.x() << del << pose.y() << del << pose.getAngle().deg();

	writeToLog(ss.str());
}

void EventLogger::writeToLog(const string &toWrite) const
{
	if (!_started)
		return;
	lock_guard<mutex> guard(writeLock);

	auto timeNow = chrono::system_clock::now();
	long long ms = chrono::duration_cast<chrono::milliseconds>(timeNow.time_since_epoch()).count();

	_log << toWrite << del << ms << endl;

	if (_log.bad() || _log.fail()) {
		cerr << "failed to write to log is bad" << endl;
	}
}

void EventLogger::logTimeTaken(const string &name, long int duration) const
{
	stringstream ss;
	ss << "TIMETAKEN" << del << name << del << duration;

	writeToLog(ss.str());
}

void EventLogger::stop() const
{
	_started = false;
	_log.close();
}

void EventLogger::start() const
{
	if (!_started) {
		_log.open(_filename);
		cout << "opening log " << _log.is_open() << " at: " << _filename << endl;
		if (!_log.is_open()) {
			cerr << "failed to open log, logging disabled";
			_started = false;
		} else {
			_started = true;
		}
	}
}

EventLogger *EventLogger::logger = NULL;
