#pragma once

#include "Decoder.h"
#include "meta/ISensorConfig.h"
#include <vector>

namespace taco
{
/**
 * The Roadsigndecoder manages the decoding of a road sign struct.
 */
class RoadSignDecoder : public Decoder
{
  public:
	RoadSignDecoder(IEventLogger::ConstPtr logger);
	virtual ~RoadSignDecoder();

	virtual std::vector<IPerceptor::ConstPtr> decode(
			const InputPin &pin, adtf::IMediaTypeDescription *mediaTypeDescription, adtf::IMediaSample *mediaSample);

  protected:
	/** The name of the represented road sign perceptor. */
	std::string _perceptiorName;
};
}
