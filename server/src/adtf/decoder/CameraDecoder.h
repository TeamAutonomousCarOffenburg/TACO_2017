#pragma once

#include "Decoder.h"
#include "meta/ICameraConfig.h"

#include <vector>

namespace taco
{
/**
 * The CameraDecoder manages the decoding of an image pin.
 *
 * \author Stefan Glaser
 */
class CameraDecoder : public Decoder
{
  public:
	CameraDecoder(ICameraConfig::ConstPtr config, IEventLogger::ConstPtr logger);
	virtual ~CameraDecoder();

	virtual std::vector<IPerceptor::ConstPtr> decode(const taco::InputPin &pin,
			adtf::IMediaTypeDescription *mediaTypeDescription, adtf::IMediaSample *mediaSample);

  protected:
	/** The name of the represented camera perceptor. */
	std::string _perceptorName;

	/** The time of the measurement. */
	long _time;

	/** The width of the camera frame. */
	unsigned int _frameWidth;

	/** The height of the camera frame. */
	unsigned int _frameHeight;
};
}
