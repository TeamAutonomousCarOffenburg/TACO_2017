#pragma once

#include "../CameraConfig.h"
#include "SensorConfigFactory.h"
// Factory für eine CameraConfig.
//Überschreibt die Int- und Double-"Wertfunktionen".
class CameraConfigFactory : public SensorConfigFactory
{
  protected:
	unsigned int width;
	unsigned int height;
	float focalPointX;
	float focalPointY;
	float focalLengthX;
	float focalLengthY;

  public:
	virtual ~CameraConfigFactory(){};

	virtual taco::SensorConfig::ConstPtr getConfig()
	{
		return boost::make_shared<const taco::CameraConfig>(name, vec.getConfig(), ang.getConfig(), width, height,
				focalPointX, focalPointY, focalLengthX, focalLengthY);
	}

	virtual bool Double(std::vector<std::string> &context, double value)
	{
		if (context.back().compare("focalPointX") == 0) {
			focalPointX = (float) value;
			return true;
		}
		if (context.back().compare("focalPointY") == 0) {
			focalPointY = (float) value;
			return true;
		}
		if (context.back().compare("focalLengthX") == 0) {
			focalLengthX = (float) value;
			return true;
		}
		if (context.back().compare("focalLengthY") == 0) {
			focalLengthY = (float) value;
			return true;
		}
		// Context definiert keinen Wert der CameraConfig.
		//->Rufe Elternfunktion auf da es sich wahrscheinlich um einen Wert der SensorConfig handelt.
		return SensorConfigFactory::Double(context, value);
	}

	virtual bool Int(std::vector<std::string> &context, int value)
	{
		if (context.back().compare("width") == 0) {
			width = (unsigned) value;
			return true;
		}
		if (context.back().compare("height") == 0) {
			height = (unsigned) value;
			return true;
		}
		// Context definiert keinen Wert der CameraConfig.
		//->Rufe Elternfunktion auf da es sich wahrscheinlich um einen Wert der SensorConfig handelt.
		return SensorConfigFactory::Int(context, value);
	}
};
