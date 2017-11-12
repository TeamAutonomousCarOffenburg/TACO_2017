#pragma once

#include "Decoder.h"
#include "meta/ISensorConfig.h"

#include <vector>

namespace taco
{
/**
 * The WheelTickDecoder manages the decoding a wheel tick-pin.
 *
 * \author Stefan Glaser
 */
class WheelTickDecoder : public Decoder
{
  public:
	WheelTickDecoder(ISensorConfig::ConstPtr config, IEventLogger::ConstPtr logger);
	virtual ~WheelTickDecoder();

	virtual std::vector<IPerceptor::ConstPtr> decode(const taco::InputPin &pin,
			adtf::IMediaTypeDescription *mediaTypeDescription, adtf::IMediaSample *mediaSample);

  protected:
	/** The name of the represented gyro perceptor. */
	std::string _perceptorName;
	int _ticks; /** The number of ticks. */
	char _sign; /** The sign (direction of rotation). */
	long _time; /** The time of the measurement. */
};
}
