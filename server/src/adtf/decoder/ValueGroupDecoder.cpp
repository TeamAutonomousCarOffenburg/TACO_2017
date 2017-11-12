#include "ValueGroupDecoder.h"
#include "perception/IValuePerceptor.h"
#include "perception/impl/ValuePerceptor.h"

using namespace taco;

ValueGroupDecoder::ValueGroupDecoder(
		const std::vector<ISensorConfig::ConstPtr> &configs, IEventLogger::ConstPtr logger, double conversionFactor)
	: Decoder(logger), _time(0), _conversionFactor(conversionFactor)
{
	for (unsigned int i = 0; i < configs.size(); i++) {
		_pins.push_back(InputPin(configs[i]->getPerceptorName(), "tSignalValue"));
	}
}

ValueGroupDecoder::ValueGroupDecoder(
		const vector<IDistanceSensorConfig::ConstPtr> &configs, IEventLogger::ConstPtr logger, double conversionFactor)
	: Decoder(logger), _time(0), _conversionFactor(conversionFactor)
{
	for (unsigned int i = 0; i < configs.size(); i++) {
		_pins.push_back(InputPin(configs[i]->getPerceptorName(), "tSignalValue"));
	}
}

ValueGroupDecoder::ValueGroupDecoder(
		const std::vector<IActuatorConfig::ConstPtr> &configs, IEventLogger::ConstPtr logger, double conversionFactor)
	: Decoder(logger), _time(0), _conversionFactor(conversionFactor)
{
	for (unsigned int i = 0; i < configs.size(); i++) {
		_pins.push_back(InputPin(configs[i]->getPerceptorName(), "tSignalValue"));
	}
}

ValueGroupDecoder::ValueGroupDecoder(
		const std::vector<IServoDriveConfig::ConstPtr> &configs, IEventLogger::ConstPtr logger, double conversionFactor)
	: Decoder(logger), _time(0), _conversionFactor(conversionFactor)
{
	for (unsigned int i = 0; i < configs.size(); i++) {
		_pins.push_back(InputPin(configs[i]->getPerceptorName(), "tSignalValue"));
	}
}

ValueGroupDecoder::~ValueGroupDecoder()
{
}

std::vector<IPerceptor::ConstPtr> ValueGroupDecoder::decode(
		const InputPin &pin, adtf::IMediaTypeDescription *mediaTypeDescription, adtf::IMediaSample *mediaSample)
{
	int pinIndex = indexOfPin(pin);

	if (pinIndex >= 0) {
		// Extract value and timestamp
		__adtf_sample_read_lock_mediadescription(mediaTypeDescription, mediaSample, mediaCoder);

		tFloat32 value = 0;
		mediaCoder->Get("f32Value", (tVoid *) &value);

		// -1 seems to mean "no value", at least on the new car
		if (value != -1) {
			value *= _conversionFactor;

			tUInt32 time = 0;
			mediaCoder->Get("ui32ArduinoTimestamp", (tVoid *) &time);
			// Update internal values

			std::string name = _pins.at(pinIndex).name;
			_logger->logDecodeEvent(time, name, value);

			long newTime = long(time);
			//    if (newTime > _time) {
			//      clearReceived();
			//    }
			_time = newTime;

			_perceptors.push_back(boost::make_shared<DoubleValuePerceptor>(pin.name, time, double(value)));
		}

		_pins[pinIndex].received = true;

		if (allPinsReceived()) {
			std::vector<IPerceptor::ConstPtr> perceptors = _perceptors;
			clearReceived();

			return perceptors;
		}
	}

	return std::vector<IPerceptor::ConstPtr>();
}

void ValueGroupDecoder::clearReceived()
{
	_perceptors.clear();
	Decoder::clearReceived();
}
