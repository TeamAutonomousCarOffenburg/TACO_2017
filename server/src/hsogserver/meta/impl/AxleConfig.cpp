#include "AxleConfig.h"

using namespace taco;
using namespace Eigen;

AxleConfig::AxleConfig(const std::string &name, const Eigen::Vector3d position, const double &length,
		const double &wheelDiameter, const Angle &wheelAngle, const std::string &leftWheelTacho,
		const std::string &rightWheelTacho)
	: _name(name), _position(position(0), 0, position(2)), _length(length), _wheelDiameter(wheelDiameter),
	  _wheelAngle(wheelAngle), _leftWheelTacho(leftWheelTacho), _rightWheelTacho(rightWheelTacho)
{
}

AxleConfig::~AxleConfig()
{
}

const std::string &AxleConfig::getName() const
{
	return _name;
}

const Eigen::Vector3d &AxleConfig::getPosition() const
{
	return _position;
}

const double &AxleConfig::getLength() const
{
	return _length;
}

const double &AxleConfig::getWheelDiameter() const
{
	return _wheelDiameter;
}

const Angle &AxleConfig::getWheelAngle() const
{
	return _wheelAngle;
}

const std::string &AxleConfig::getLeftWheelTachoName() const
{
	return _leftWheelTacho;
}

const std::string &AxleConfig::getRightWheelTachoName() const
{
	return _rightWheelTacho;
}
