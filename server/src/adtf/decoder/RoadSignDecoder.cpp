#include "RoadSignDecoder.h"
#include "AADCCar.h"
#include "perception/impl/RoadSignPerceptor.h"

using namespace taco;

RoadSignDecoder::RoadSignDecoder(IEventLogger::ConstPtr logger) : Decoder(logger), _perceptiorName(AADCCar::SIGNS)
{
	_pins.push_back(InputPin(AADCCar::SIGNS, "tRoadSignExt"));
}

RoadSignDecoder::~RoadSignDecoder()
{
}

std::vector<IPerceptor::ConstPtr> RoadSignDecoder::decode(
		const InputPin &pin, adtf::IMediaTypeDescription *mediaTypeDescription, adtf::IMediaSample *mediaSample)
{
	int pinIndex = indexOfPin(pin);

	if (pinIndex >= 0) {
		// Extract value and timestamp
		__adtf_sample_read_lock_mediadescription(mediaTypeDescription, mediaSample, mediaCoder);

		tInt16 i16Identifier;
		mediaCoder->Get("i16Identifier", (tVoid *) &i16Identifier);

		tFloat32 f32ImageSize;
		mediaCoder->Get("f32Imagesize", (tVoid *) &f32ImageSize);

		float tVec[3];
		mediaCoder->Get("af32TVec", (tVoid *) &tVec);

		float rVec[3];
		mediaCoder->Get("af32RVec", (tVoid *) &rVec);

		cv::Mat tVecMat = cv::Mat(1, 3, CV_32F, tVec);
		cv::Mat rVecMat = cv::Mat(1, 3, CV_32F, rVec);

		long time = mediaSample->GetTime();

		std::string name = _pins.at(pinIndex).name;
		_logger->logDecodeEvent(time, name, i16Identifier);

		std::vector<IPerceptor::ConstPtr> perceptors;
		perceptors.push_back(boost::make_shared<RoadSignPerceptor>(
				_perceptiorName, time, i16Identifier, f32ImageSize, tVecMat, rVecMat));

		return perceptors;
	}

	return std::vector<IPerceptor::ConstPtr>();
}
