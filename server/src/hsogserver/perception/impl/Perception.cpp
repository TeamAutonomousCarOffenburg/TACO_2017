#include "Perception.h"

using namespace taco;

Perception::Perception()
{
}

Perception::~Perception()
{
}

IPerceptor::ConstPtr Perception::getPerceptor(std::string name) const
{
	IPerceptor::ConstPtr perceptor;

	auto it = _perceptors.find(name);
	if (it != _perceptors.end()) {
		perceptor = boost::dynamic_pointer_cast<const IPerceptor>(it->second);
	}

	return perceptor;
}

IDoubleValuePerceptor::ConstPtr Perception::getDoubleValuePerceptor(std::string name) const
{
	IDoubleValuePerceptor::ConstPtr valuePerceptor;

	auto it = _perceptors.find(name);
	if (it != _perceptors.end()) {
		valuePerceptor = boost::dynamic_pointer_cast<const IDoubleValuePerceptor>(it->second);
	}

	return valuePerceptor;
}

IGyroPerceptor::ConstPtr Perception::getGyroPerceptor(std::string name) const
{
	IGyroPerceptor::ConstPtr gyroPerceptor;

	auto it = _perceptors.find(name);
	if (it != _perceptors.end()) {
		gyroPerceptor = boost::dynamic_pointer_cast<const IGyroPerceptor>(it->second);
	}

	return gyroPerceptor;
}

IAccelerometerPerceptor::ConstPtr Perception::getAccelerometerPerceptor(std::string name) const
{
	IAccelerometerPerceptor::ConstPtr accPerceptor;

	auto it = _perceptors.find(name);
	if (it != _perceptors.end()) {
		accPerceptor = boost::dynamic_pointer_cast<const IAccelerometerPerceptor>(it->second);
	}

	return accPerceptor;
}

IIMUPerceptor::ConstPtr Perception::getIMUPerceptor(std::string name) const
{
	IIMUPerceptor::ConstPtr imuPerceptor;

	auto it = _perceptors.find(name);
	if (it != _perceptors.end()) {
		imuPerceptor = boost::dynamic_pointer_cast<const IIMUPerceptor>(it->second);
	}

	return imuPerceptor;
}

IWheelTickPerceptor::ConstPtr Perception::getWheelTickPerceptor(std::string name) const
{
	IWheelTickPerceptor::ConstPtr tickPerceptor;

	auto it = _perceptors.find(name);
	if (it != _perceptors.end()) {
		tickPerceptor = boost::dynamic_pointer_cast<const IWheelTickPerceptor>(it->second);
	}

	return tickPerceptor;
}

ICameraPerceptor::ConstPtr Perception::getCameraPerceptor(std::string name) const
{
	ICameraPerceptor::ConstPtr camPerceptor;

	auto it = _perceptors.find(name);
	if (it != _perceptors.end()) {
		camPerceptor = boost::dynamic_pointer_cast<const ICameraPerceptor>(it->second);
	}

	return camPerceptor;
}

IPointCloudPerceptor::ConstPtr Perception::getPointCloudPerceptor(std::string name) const
{
	IPointCloudPerceptor::ConstPtr pclPerceptor;

	auto it = _perceptors.find(name);
	if (it != _perceptors.end()) {
		pclPerceptor = boost::dynamic_pointer_cast<const IPointCloudPerceptor>(it->second);
	}

	return pclPerceptor;
}

IJuryPerceptor::ConstPtr Perception::getJuryPerceptor(std::string name) const
{
	IJuryPerceptor::ConstPtr juryPerceptor;

	auto it = _perceptors.find(name);
	if (it != _perceptors.end()) {
		juryPerceptor = boost::dynamic_pointer_cast<const IJuryPerceptor>(it->second);
	}

	return juryPerceptor;
}

IManeuverListPerceptor::ConstPtr Perception::getManeuverListPerceptor(std::string name) const
{
	IManeuverListPerceptor::ConstPtr manListPerceptor;

	auto it = _perceptors.find(name);
	if (it != _perceptors.end()) {
		manListPerceptor = boost::dynamic_pointer_cast<const IManeuverListPerceptor>(it->second);
	}

	return manListPerceptor;
}

const bool Perception::nextPerceptorMap()
{
	std::lock_guard<std::mutex> lock(_queueMutex);
	int queueSize = _perceptorQueue.size();

	// Check failure (no new perceptor-map available)
	if (queueSize == 0) {
		return false;
	}

	// Progress to the next perceptor-map
	_perceptors.clear();
	_perceptors = _perceptorQueue.front();
	_perceptorQueue.pop_front();
	queueSize--;

	// Try to merge as many follow up perceptor-maps as possible into the current one
	bool duplicate = false;
	while (!duplicate && queueSize > 0) {
		// Search for duplicate perceptors
		for (auto it = _perceptors.begin(); it != _perceptors.end(); it++) {
			if (_perceptorQueue.front().count(it->first) > 0) {
				// Found a perceptor with the same name
				duplicate = true;
				break;
			}
		}

		if (!duplicate) {
			// Merge first map in queue with internal perceptor-map
			_perceptors.insert(_perceptorQueue.front().begin(), _perceptorQueue.front().end());

			// Pop front element of queue
			_perceptorQueue.pop_front();
			queueSize--;
		}
	}
	return true;
}

void Perception::updatePerceptors(std::map<std::string, IPerceptor::ConstPtr> &newPerceptors)
{
	std::lock_guard<std::mutex> lock(_queueMutex);
	_perceptorQueue.push_back(newPerceptors);
}
