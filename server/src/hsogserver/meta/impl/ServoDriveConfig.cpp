#include "ServoDriveConfig.h"

using namespace taco;

ServoDriveConfig::ServoDriveConfig(const std::string &name, const double &min, const double &max)
	: ServoDriveConfig(name, name, name, min, max)
{
}

ServoDriveConfig::ServoDriveConfig(const std::string &name, const std::string &perceptorName,
		const std::string &effectorName, const double &min, const double &max)
	: ActuatorConfig(name, perceptorName, effectorName), _minPosition(min), _maxPosition(max)
{
}

ServoDriveConfig::~ServoDriveConfig()
{
}

const double &ServoDriveConfig::getMinPosition() const
{
	return _minPosition;
}

const double &ServoDriveConfig::getMaxPosition() const
{
	return _maxPosition;
}
