#pragma once

#include "Decoder.h"
#include "meta/ISensorConfig.h"

#include <vector>

namespace taco
{
/**
 * The ManeuverListPerceptor manages the decoding a maneuver list.
 *
 * \author Stefan Glaser
 */
class ManeuverListDecoder : public Decoder
{
  public:
	ManeuverListDecoder(IEventLogger::ConstPtr logger);
	virtual ~ManeuverListDecoder();

	virtual std::vector<IPerceptor::ConstPtr> decode(
			const InputPin &pin, adtf::IMediaTypeDescription *mediaTypeDescription, adtf::IMediaSample *mediaSample);

  protected:
	/** The name of the represented maneuver list perceptor. */
	std::string _perceptorName;
};
}
