#pragma once

#include "Decoder.h"
#include "meta/ISensorConfig.h"

#include <vector>

namespace taco
{
/**
 * The GyroDecoder manages the decoding a gyro-pin group.
 *
 * \author Stefan Glaser
 */
class GyroDecoder : public Decoder
{
  public:
	GyroDecoder(ISensorConfig::ConstPtr config, IEventLogger::ConstPtr logger);
	virtual ~GyroDecoder();

	virtual std::vector<IPerceptor::ConstPtr> decode(
			const InputPin &pin, adtf::IMediaTypeDescription *mediaTypeDescription, adtf::IMediaSample *mediaSample);

  protected:
	/** The name of the represented gyro perceptor. */
	std::string _perceptorName;
	double _w;  /** The w component of the gyro quaternion. */
	double _x;  /** The x component of the gyro quaternion. */
	double _y;  /** The y component of the gyro quaternion. */
	double _z;  /** The z component of the gyro quaternion. */
	long _time; /** The time of the measurement. */
};
}
