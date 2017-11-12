#include "WheelTickDecoder.h"
#include "perception/IWheelTickPerceptor.h"
#include "perception/impl/WheelTickPerceptor.h"

using namespace taco;

WheelTickDecoder::WheelTickDecoder(ISensorConfig::ConstPtr config, IEventLogger::ConstPtr logger)
	: Decoder(logger), _perceptorName(config->getPerceptorName()), _time(0)
{
	_pins.push_back(InputPin(_perceptorName, "tWheelData"));
}

WheelTickDecoder::~WheelTickDecoder()
{
}

std::vector<IPerceptor::ConstPtr> WheelTickDecoder::decode(
		const InputPin &pin, adtf::IMediaTypeDescription *mediaTypeDescription, adtf::IMediaSample *mediaSample)
{
	int pinIndex = indexOfPin(pin);

	if (pinIndex >= 0) {
		// Extract value and timestamp
		__adtf_sample_read_lock_mediadescription(mediaTypeDescription, mediaSample, mediaCoder);

		tUInt32 value = 0;
		mediaCoder->Get("ui32WheelTach", (tVoid *) &value);

		tInt8 dir = 0;
		mediaCoder->Get("i8WheelDir", (tVoid *) &dir);

		tUInt32 time = 0;
		mediaCoder->Get("ui32ArduinoTimestamp", (tVoid *) &time);

		std::vector<IPerceptor::ConstPtr> perceptors;
		perceptors.push_back(
				boost::make_shared<WheelTickPerceptor>(_perceptorName, long(time), int(value), dir == 0 ? 1 : -1));

		return perceptors;
	}

	return std::vector<IPerceptor::ConstPtr>();
}
