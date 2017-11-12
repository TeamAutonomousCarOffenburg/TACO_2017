#include "DistanceSensorConfig.h"

using namespace taco;

DistanceSensorConfig::DistanceSensorConfig(const std::string &name, const Eigen::Vector3d &position,
		const Eigen::AngleAxisd &orientation, const double &minDistance, const double &maxDistance,
		const double &scanWidth)
	: DistanceSensorConfig(name, name, position, orientation, minDistance, maxDistance, scanWidth)
{
}

DistanceSensorConfig::DistanceSensorConfig(const std::string &name, const std::string &perceptorName,
		const Eigen::Vector3d &position, const Eigen::AngleAxisd &orientation, const double &minDistance,
		const double &maxDistance, const double &scanWidth)
	: SensorConfig(name, name, position, orientation), _minDistance(minDistance), _maxDistance(maxDistance),
	  _scanWidth(scanWidth)
{
}

DistanceSensorConfig::~DistanceSensorConfig()
{
}

const double &DistanceSensorConfig::getMinDistance() const
{
	return _minDistance;
}

const double &DistanceSensorConfig::getMaxDistance() const
{
	return _maxDistance;
}

const double &DistanceSensorConfig::getScanWidth() const
{
	return _scanWidth;
}
