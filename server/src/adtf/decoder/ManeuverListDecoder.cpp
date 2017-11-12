#include "ManeuverListDecoder.h"
#include "AADCCar.h"
#include "perception/impl/ManeuverListPerceptor.h"
#include "utils/driveinstructionreader/DriveInstructionReader.h"

using namespace taco;

ManeuverListDecoder::ManeuverListDecoder(IEventLogger::ConstPtr logger)
	: Decoder(logger), _perceptorName(AADCCar::MANEUVER_LIST)
{
	_pins.push_back(InputPin(AADCCar::MANEUVER_LIST, "tManeuverList"));
}

ManeuverListDecoder::~ManeuverListDecoder()
{
}

std::vector<IPerceptor::ConstPtr> ManeuverListDecoder::decode(
		const InputPin &pin, adtf::IMediaTypeDescription *mediaTypeDescription, adtf::IMediaSample *mediaSample)
{
	int pinIndex = indexOfPin(pin);

	if (pinIndex >= 0) {
		// Extract value and timestamp
		__adtf_sample_read_lock_mediadescription(mediaTypeDescription, mediaSample, mediaCoder);

		tSize size = 0;
		mediaCoder->Get("i32Size", (tVoid *) &size);

		std::vector<tSize> vecDynamicIDs;

		// create a buffer depending on the size element
		tChar *pcBuffer = new tChar[size];
		vecDynamicIDs.resize(size);
		// get the dynamic ids (we already got the first "static" size element)
		mediaCoder->GetDynamicBufferIDs(&(vecDynamicIDs.front()), size);
		// iterate over all elements
		for (tUInt32 i = 0; i < vecDynamicIDs.size(); ++i) {
			// get the value and put it into the buffer
			mediaCoder->Get(vecDynamicIDs[i], (tVoid *) &pcBuffer[i]);
		}

		// set the resulting char buffer to the string object
		string xml_string = string((const tChar *) pcBuffer);

		// Fetch the list of drive instructions
		std::vector<Maneuver> driveInstructions;
		DriveInstructionReader::loadFromString(xml_string, driveInstructions);

		// cleanup the buffer
		delete pcBuffer;

		std::vector<IPerceptor::ConstPtr> perceptors;
		perceptors.push_back(boost::make_shared<ManeuverListPerceptor>(_perceptorName, 0, driveInstructions));

		return perceptors;
	}
	return std::vector<IPerceptor::ConstPtr>();
}