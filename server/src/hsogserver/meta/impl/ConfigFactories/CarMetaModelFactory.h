#pragma once

#include "IConfigFactory.h"
#include "string"

// Factory für den Konstruktor der Elternklasse CarMetaModel.
class CarMetaModelFactory : public IConfigFactory
{
  public:
	double floorHeight;
	std::string steeringServoName;
	std::string mainMotorName;

	virtual ~CarMetaModelFactory(){

	};

	virtual bool String(std::vector<std::string> &context, const char *value)
	{
		if (context.back().compare("steeringServoName") == 0) {
			steeringServoName = value;
			return true;
		}
		if (context.back().compare("mainMotorName") == 0) {
			mainMotorName = value;
			return true;
		}
		return false;
	};

	virtual bool Double(std::vector<std::string> &context, double value)
	{
		if (context.back().compare("floorHeight") == 0) {
			floorHeight = value;
			return true;
		}
		return false;
	};

	virtual bool Int(std::vector<std::string> &context, int value)
	{
		return false;
	};
};
