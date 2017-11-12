#pragma once

#include "Decoder.h"
#include "meta/ICameraConfig.h"

#include <vector>

namespace taco
{
/**
 * The DepthImageDecoder manages the decoding of a depth image pin.
 *
 * \author Stefan Glaser
 */
class DepthImageDecoder : public Decoder
{
  public:
	DepthImageDecoder(ICameraConfig::ConstPtr config, IEventLogger::ConstPtr logger);
	virtual ~DepthImageDecoder();

	virtual std::vector<IPerceptor::ConstPtr> decode(
			const InputPin &pin, adtf::IMediaTypeDescription *mediaTypeDescription, adtf::IMediaSample *mediaSample);

  protected:
	/** The name of the represented gyro perceptor. */
	std::string _perceptorName;

	/** The time of the measurement. */
	long _time;

	/** The width of the camera image. */
	unsigned int _frameWidth;

	/** The height of the camera image. */
	unsigned int _frameHeight;

	double _focalLengthX;
	double _focalLengthY;
};
}
