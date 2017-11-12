#include "ActuatorConfig.h"

using namespace taco;

ActuatorConfig::ActuatorConfig(const std::string &name) : ActuatorConfig(name, name, name)
{
}

ActuatorConfig::ActuatorConfig(
		const std::string &name, const std::string &perceptorName, const std::string &effectorName)
	: _name(name), _perceptorName(perceptorName), _effectorName(effectorName)
{
}

ActuatorConfig::~ActuatorConfig()
{
}

const std::string &ActuatorConfig::getName() const
{
	return _name;
}

const std::string &ActuatorConfig::getPerceptorName() const
{
	return _perceptorName;
}

const std::string &ActuatorConfig::getEffectorName() const
{
	return _effectorName;
}
