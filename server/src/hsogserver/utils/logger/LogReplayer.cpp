#include "LogReplayer.h"

using namespace std;
using namespace taco;

LogReplayer::LogReplayer(string logPath, IPerception::Ptr perception)
{
	_log.open(logPath);
	if (!_log.is_open()) {
		cerr << "failed to open log for replay: " << logPath << endl;
	}
	_perception = perception;
}

void LogReplayer::load()
{
	while (!_log.eof()) {
		string line;
		getline(_log, line);
		if (line.empty())
			break;

		IPerceptor::ConstPtr perceptor = createPerceptor(line);
		if (perceptor)
			perceptors.push_back(perceptor);
		_loaded = true;
	}
}
int LogReplayer::getFirstTime()
{
	if (perceptors.empty()) {
		return 0;
	}
	return perceptors[0]->getTime();
}

bool LogReplayer::next(int thisTime)
{
	if (perceptors.empty()) {
		cerr << "no perceptors to replay" << endl;
		return false;
	}

	map<string, IPerceptor::ConstPtr> perceptorMap;
	unsigned int i = 0;
	for (; i < perceptors.size(); i++) {
		int perceptorTime = perceptors[i]->getTime();

		if (perceptorTime > thisTime) {
			break;
		}

		perceptorMap.insert(make_pair(perceptors[i]->getName(), perceptors[i]));
	}

	// delete from perceptors
	if (i > 0) {
		perceptors.erase(perceptors.begin(), perceptors.begin() + i);
		_perception->updatePerceptors(perceptorMap);
	}

	return true;
}

IPerceptor::ConstPtr LogReplayer::createPerceptor(string line)
{
	vector<string> splitted = split(line);
	IPerceptor::ConstPtr perceptor;

	string name = splitted[1];

	std::string::size_type sz;
	long time = stol(splitted[2], &sz);
	double value = stod(splitted[3], &sz);

	if ((name.find("IR") != string::npos) || (name.find("US") != string::npos) || (name.find("WH") != string::npos)) {
		perceptor = IDoubleValuePerceptor::ConstPtr(new DoubleValuePerceptor(name, time, value));
	} else if (name.find("Gyro") != string::npos) {
		GyroPerceptor *gyr = generateGyroPerceptor(name, time, value);
		if (gyr != nullptr) {
			perceptor = IGyroPerceptor::ConstPtr(gyr);
		}
	} else if (name.find("Acc") != string::npos) {
		AccelerometerPerceptor *acc = generateAccelerometerPerceptor(name, time, value);
		if (acc != nullptr) {
			perceptor = IAccelerometerPerceptor::ConstPtr(acc);
		}
	}

	return perceptor;
}

AccelerometerPerceptor *LogReplayer::generateAccelerometerPerceptor(string name, long time, double value)
{
	static bool xset, yset, zset;
	static double xvalue, yvalue, zvalue;

	if (name.find("x") != string::npos) {
		xvalue = value;
		xset = true;
	} else if (name.find("y") != string::npos) {
		yvalue = value;
		yset = true;
	} else if (name.find("z") != string::npos) {
		zvalue = value;
		zset = true;
	}

	if (xset && yset && zset) {
		xset = yset = zset = false;

		return new AccelerometerPerceptor(name, time, xvalue, yvalue, zvalue);
	}
	return nullptr;
}

GyroPerceptor *LogReplayer::generateGyroPerceptor(string name, long time, double value)
{
	static bool xset, yset, zset, wset;
	static double xvalue, yvalue, zvalue, wvalue;

	if (name.find("x") != string::npos) {
		xvalue = value;
		xset = true;
	} else if (name.find("y") != string::npos) {
		yvalue = value;
		yset = true;
	} else if (name.find("z") != string::npos) {
		zvalue = value;
		zset = true;
	} else if (name.find("w") != string::npos) {
		wvalue = value;
		wset = true;
	}

	if (xset && yset && zset && wset) {
		xset = yset = zset = wset = false;

		return new GyroPerceptor(name, time, wvalue, xvalue, yvalue, zvalue);
	}
	return nullptr;
}

vector<string> &LogReplayer::split(const string &s, vector<string> &elems)
{
	stringstream ss(s);
	string item;
	while (getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}

vector<string> LogReplayer::split(const string &s)
{
	vector<string> elems;
	split(s, elems);
	return elems;
}
