#pragma once

#include "../../IAxleConfig.h"
#include "../AxleConfig.h"
#include "ConfigFactory.h"
#include "Vector3DFactory.h"

// Factory für eine AxleConfig
class AxleConfigFactory : public ConfigFactory<taco::IAxleConfig::ConstPtr>
{
  protected:
	std::string name;
	Vector3DFactory pos;
	double length;
	double wheelDiameter;
	double wheelAngle;
	std::string leftWheelTacho;
	std::string rightWheelTacho;

  public:
	virtual taco::IAxleConfig::ConstPtr getConfig()
	{
		return boost::make_shared<const taco::AxleConfig>(name, pos.getConfig(), length, wheelDiameter,
				taco::Angle::deg(wheelAngle), leftWheelTacho, rightWheelTacho);
	};

	virtual ~AxleConfigFactory(){};

	virtual bool String(std::vector<std::string> &context, const char *value)
	{
		if (context.back().compare("name") == 0) {
			name = value;
			return true;
		}
		if (context.back().compare("leftWheelTacho") == 0) {
			leftWheelTacho = value;
			return true;
		}
		if (context.back().compare("rightWheelTacho") == 0) {
			rightWheelTacho = value;
			return true;
		}
		return false;
	};

	virtual bool Double(std::vector<std::string> &context, double value)
	{
		if (context.back().compare("length") == 0) {
			length = value;
			return true;
		}
		if (context.back().compare("wheelDiameter") == 0) {
			wheelDiameter = value;
			return true;
		}
		if (context.back().compare("wheelAngle") == 0) {
			wheelAngle = value;
			return true;
		}
		return pos.Double(context, value);
	};
};
