#pragma once

#include "Decoder.h"
#include "InputPin.h"
#include "meta/ICarMetaModel.h"
#include "perception/IPerception.h"

#include "utils/logger/IEventLogger.h"

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <boost/smart_ptr.hpp>
#include <vector>

namespace taco
{
/**
 * The ADTFPinMessageDecoder class provides the mapping from input pins
 * of the ADTF-Filter to internal Perceptor objects.
 *
 * \author Stefan Glaser
 */
class ADTFPinMessageDecoder
{
  public:
	ADTFPinMessageDecoder(
			IPerception::Ptr perception, ICarMetaModel::ConstPtr carMetaModel, IEventLogger::ConstPtr logger);
	virtual ~ADTFPinMessageDecoder();

	typedef boost::shared_ptr<ADTFPinMessageDecoder> Ptr;
	typedef boost::shared_ptr<const ADTFPinMessageDecoder> ConstPtr;

	virtual const std::vector<taco::InputPin> getPinConfigs();
	virtual bool decode(const taco::InputPin &pin, adtf::IMediaTypeDescription *mediaTypeDescription,
			adtf::IMediaSample *mediaSample);

  protected:
	taco::IPerception::Ptr _perception;
	std::vector<taco::DecoderPtr> _decoder;
};
}
