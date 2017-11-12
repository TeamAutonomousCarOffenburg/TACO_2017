#include "SensorConfig.h"

using namespace taco;

SensorConfig::SensorConfig(
		const std::string &name, const Eigen::Vector3d &position, const Eigen::AngleAxisd &orientation)
	: SensorConfig(name, name, position, orientation)
{
}

SensorConfig::SensorConfig(const std::string &name, const std::string &perceptorName, const Eigen::Vector3d &position,
		const Eigen::AngleAxisd &orientation)
	: _name(name), _perceptorName(perceptorName), _pose(position, orientation)
{
}

SensorConfig::~SensorConfig()
{
}

const std::string &SensorConfig::getName() const
{
	return _name;
}

const std::string &SensorConfig::getPerceptorName() const
{
	return _perceptorName;
}

const Pose3D &SensorConfig::getPose() const
{
	return _pose;
}
