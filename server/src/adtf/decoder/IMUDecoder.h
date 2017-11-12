#pragma once

#include "Decoder.h"
#include "meta/ISensorConfig.h"

#include <vector>

namespace taco
{
/**
 * The IMUDecoder manages the decoding a IMU-pin.
 *
 * \author Stefan Glaser
 */
class IMUDecoder : public Decoder
{
  public:
	IMUDecoder(ISensorConfig::ConstPtr config, IEventLogger::ConstPtr logger);
	virtual ~IMUDecoder();

	virtual std::vector<IPerceptor::ConstPtr> decode(const taco::InputPin &pin,
			adtf::IMediaTypeDescription *mediaTypeDescription, adtf::IMediaSample *mediaSample);

  protected:
	/** The name of the represented gyro perceptor. */
	std::string _perceptorName;
};
}
