#pragma once

#include "Decoder.h"
#include "meta/IActuatorConfig.h"
#include "meta/IDistanceSensorConfig.h"
#include "meta/ISensorConfig.h"
#include "meta/IServoDriveConfig.h"

#include <vector>

namespace taco
{
/**
 * The ValueGroupDecoder manages a group of value-pins.
 *
 * \author Stefan Glaser
 */
class ValueGroupDecoder : public Decoder
{
  public:
	ValueGroupDecoder(const std::vector<ISensorConfig::ConstPtr> &configs, IEventLogger::ConstPtr logger,
			double conversionFactor = 1);
	ValueGroupDecoder(const std::vector<IDistanceSensorConfig::ConstPtr> &configs, IEventLogger::ConstPtr logger,
			double conversionFactor = 1);
	ValueGroupDecoder(const std::vector<IActuatorConfig::ConstPtr> &configs, IEventLogger::ConstPtr logger,
			double conversionFactor = 1);
	ValueGroupDecoder(const std::vector<IServoDriveConfig::ConstPtr> &configs, IEventLogger::ConstPtr logger,
			double conversionFactor = 1);
	virtual ~ValueGroupDecoder();

	virtual std::vector<IPerceptor::ConstPtr> decode(
			const InputPin &pin, adtf::IMediaTypeDescription *mediaTypeDescription, adtf::IMediaSample *mediaSample);

	virtual void clearReceived();

  protected:
	long _time; /** The time of the measurement. */
	std::vector<IPerceptor::ConstPtr> _perceptors;
	double _conversionFactor = 1;
};
}
