#pragma once

#include "../../IDistanceSensorConfig.h"
#include "SensorConfigFactory.h"
#include <meta/impl/DistanceSensorConfig.h>

// Factory für eine DistanceSensorConfig.
//Überschreibt die Double-"Wertfunktion".
class DistanceSensorConfigFactory : public SensorConfigFactory
{
	double minDistance;
	double maxDistance;
	double scanWidth;

  public:
	virtual ~DistanceSensorConfigFactory()
	{
	}

	virtual taco::SensorConfig::ConstPtr getConfig()
	{
		return boost::make_shared<const taco::DistanceSensorConfig>(
				name, vec.getConfig(), ang.getConfig(), minDistance, maxDistance, scanWidth);
	}

	virtual bool Double(std::vector<std::string> &context, double value)
	{
		if (context.back().compare("minDistance") == 0) {
			minDistance = value;
			return true;
		}
		if (context.back().compare("maxDistance") == 0) {
			maxDistance = value;
			return true;
		}
		if (context.back().compare("scanWidth") == 0) {
			scanWidth = value;
			return true;
		}
		// Context definiert keinen Wert der DistanceSensorConfig.
		//->Rufe Elternfunktion auf da es sich wahrscheinlich um einen Wert der SensorConfig handelt.
		return SensorConfigFactory::Double(context, value);
	}
};
