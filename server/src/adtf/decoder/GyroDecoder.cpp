#include "GyroDecoder.h"
#include "perception/IGyroPerceptor.h"
#include "perception/impl/GyroPerceptor.h"

#include <eigen3/Eigen/Dense>

using namespace taco;

GyroDecoder::GyroDecoder(ISensorConfig::ConstPtr config, IEventLogger::ConstPtr logger)
	: Decoder(logger), _perceptorName(config->getPerceptorName()), _time(0)
{
	_pins.push_back(InputPin(_perceptorName + "_quat_w", "tSignalValue"));
	_pins.push_back(InputPin(_perceptorName + "_quat_x", "tSignalValue"));
	_pins.push_back(InputPin(_perceptorName + "_quat_y", "tSignalValue"));
	_pins.push_back(InputPin(_perceptorName + "_quat_z", "tSignalValue"));
}

GyroDecoder::~GyroDecoder()
{
}

std::vector<IPerceptor::ConstPtr> GyroDecoder::decode(
		const InputPin &pin, adtf::IMediaTypeDescription *mediaTypeDescription, adtf::IMediaSample *mediaSample)
{
	int pinIndex = indexOfPin(pin);

	if (pinIndex >= 0) {
		// Extract value and timestamp
		__adtf_sample_read_lock_mediadescription(mediaTypeDescription, mediaSample, mediaCoder);

		tFloat32 value = 0;
		mediaCoder->Get("f32Value", (tVoid *) &value);

		tUInt32 time = 0;
		mediaCoder->Get("ui32ArduinoTimestamp", (tVoid *) &time);

		std::string name = _pins.at(pinIndex).name;
		_logger->logDecodeEvent(time, name, value);

		// Update internal values
		long newTime = long(time);
		if (newTime > _time) {
			clearReceived();
		}
		_time = newTime;

		switch (pinIndex) {
		case 0:
			_w = double(value);
			break;
		case 1:
			_x = double(value);
			break;
		case 2:
			_y = double(value);
			break;
		case 3:
			_z = double(value);
			break;
		default:
			break;
		}
		_pins[pinIndex].received = true;

		if (allPinsReceived()) {
			std::vector<IPerceptor::ConstPtr> perceptors;
			perceptors.push_back(boost::make_shared<GyroPerceptor>(_perceptorName, _time, _w, _x, _y, _z));
			clearReceived();

			return perceptors;
		}
	}

	return std::vector<IPerceptor::ConstPtr>();
}
