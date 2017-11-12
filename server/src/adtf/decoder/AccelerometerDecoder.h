#pragma once

#include "Decoder.h"
#include "meta/ISensorConfig.h"

#include <vector>

namespace taco
{
/**
 * The AccelerometerDecoder manages the decoding a accelerometer-pin group.
 *
 * \author Stefan Glaser
 */
class AccelerometerDecoder : public Decoder
{
  public:
	AccelerometerDecoder(ISensorConfig::ConstPtr config, IEventLogger::ConstPtr logger);
	virtual ~AccelerometerDecoder();

	virtual std::vector<IPerceptor::ConstPtr> decode(const taco::InputPin &pin,
			adtf::IMediaTypeDescription *mediaTypeDescription, adtf::IMediaSample *mediaSample);

  protected:
	/** The name of the represented gyro perceptor. */
	std::string _perceptorName;
	double _x;  /** The x component of the acceleration vector. */
	double _y;  /** The y component of the acceleration vector. */
	double _z;  /** The z component of the acceleration vector. */
	long _time; /** The time of the measurement. */
};
}
