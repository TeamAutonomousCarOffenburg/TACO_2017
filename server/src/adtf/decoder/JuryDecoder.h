#pragma once

#include "Decoder.h"
#include "meta/ISensorConfig.h"

#include <vector>

namespace taco
{
/**
 * The JuryDecoder manages the decoding a jury struct.
 *
 */
class JuryDecoder : public Decoder
{
  public:
	JuryDecoder(IEventLogger::ConstPtr logger);
	virtual ~JuryDecoder();

	virtual std::vector<IPerceptor::ConstPtr> decode(
			const InputPin &pin, adtf::IMediaTypeDescription *mediaTypeDescription, adtf::IMediaSample *mediaSample);

  protected:
	/** The name of the represented jury perceptor. */
	std::string _perceptorName;
};
}
