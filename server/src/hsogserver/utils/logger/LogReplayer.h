#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include "perception/IPerception.h"
#include "perception/impl/AccelerometerPerceptor.h"
#include "perception/impl/GyroPerceptor.h"
#include "perception/impl/ValuePerceptor.h"

class LogReplayer
{
  public:
	LogReplayer(std::string logPath, taco::IPerception::Ptr perception);

	void load();

	bool next(int time);

	int getFirstTime();

  private:
	std::vector<taco::IPerceptor::ConstPtr> perceptors;
	std::ifstream _log;
	char delim = ';';
	taco::IPerception::Ptr _perception;

	bool _loaded = false;

	taco::IPerceptor::ConstPtr createPerceptor(std::string line);

	taco::AccelerometerPerceptor *generateAccelerometerPerceptor(std::string name, long time, double value);
	taco::GyroPerceptor *generateGyroPerceptor(std::string name, long time, double value);

	std::vector<std::string> &split(const std::string &s, std::vector<std::string> &elems);
	std::vector<std::string> split(const std::string &s);
};
