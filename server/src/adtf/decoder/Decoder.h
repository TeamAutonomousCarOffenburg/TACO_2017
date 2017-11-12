#pragma once

#include "InputPin.h"
#include "perception/IPerceptor.h"
#include "utils/logger/IEventLogger.h"

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <boost/smart_ptr.hpp>

namespace taco
{
/**
 * The Decoder class is the base class for all pin-decoders.
 *
 * \author Stefan Glaser
 */
class Decoder
{
  public:
	Decoder(IEventLogger::ConstPtr logger);
	virtual ~Decoder();

	virtual int indexOfPin(const InputPin &pin) const;
	virtual const std::vector<InputPin> &getPinConfig() const;

	virtual std::vector<IPerceptor::ConstPtr> decode(const taco::InputPin &pin,
			adtf::IMediaTypeDescription *mediaTypeDescription, adtf::IMediaSample *mediaSample) = 0;

  protected:
	virtual void clearReceived();
	virtual bool allPinsReceived();

	std::vector<InputPin> _pins;
	IEventLogger::ConstPtr _logger;
};

typedef boost::shared_ptr<Decoder> DecoderPtr;
}
