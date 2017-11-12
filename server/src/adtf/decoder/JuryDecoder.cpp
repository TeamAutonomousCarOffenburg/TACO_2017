#include "JuryDecoder.h"
#include "AADCCar.h"
#include "perception/impl/JuryPerceptor.h"

using namespace taco;

JuryDecoder::JuryDecoder(IEventLogger::ConstPtr logger) : Decoder(logger), _perceptorName(AADCCar::JURY_COMMAND)
{
	_pins.push_back(InputPin(AADCCar::JURY_COMMAND, "tJuryStruct"));
}

JuryDecoder::~JuryDecoder()
{
}

std::vector<IPerceptor::ConstPtr> JuryDecoder::decode(
		const InputPin &pin, adtf::IMediaTypeDescription *mediaTypeDescription, adtf::IMediaSample *mediaSample)
{
	int pinIndex = indexOfPin(pin);

	if (pinIndex >= 0) {
		// Extract value and timestamp
		__adtf_sample_read_lock_mediadescription(mediaTypeDescription, mediaSample, mediaCoder);

		tInt8 actionID;
		mediaCoder->Get("i8ActionID", (tVoid *) &actionID);

		tInt16 maneuverEntry;
		mediaCoder->Get("i16ManeuverEntry", (tVoid *) &maneuverEntry);

		long time = mediaSample->GetTime();

		std::string name = _pins.at(pinIndex).name;
		_logger->logDecodeEvent(time, name, maneuverEntry);

		std::vector<IPerceptor::ConstPtr> perceptors;
		perceptors.push_back(boost::make_shared<JuryPerceptor>(_perceptorName, time, actionID, maneuverEntry));

		return perceptors;
	}

	return std::vector<IPerceptor::ConstPtr>();
}
