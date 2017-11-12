#pragma once

#include "../ServoDriveConfig.h"
#include "ActuatorConfigFactory.h"

// Factory für eine ServoDriveConfig.
//Überschreibt die Double-"Wertfunktion".
class ServoDriveConfigFactory : public ActuatorConfigFactory
{
  protected:
	double min;
	double max;

  public:
	virtual ~ServoDriveConfigFactory(){};

	virtual taco::IActuatorConfig::ConstPtr getConfig()
	{
		return boost::make_shared<const taco::ServoDriveConfig>(name, min, max);
	}

	virtual bool Double(std::vector<std::string> &context, double value)
	{
		if (context.back().compare("min") == 0) {
			min = value;
			return true;
		}
		if (context.back().compare("max") == 0) {
			max = value;
			return true;
		}
		return false;
	};
};
