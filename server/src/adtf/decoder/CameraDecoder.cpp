#include "CameraDecoder.h"
#include "perception/ICameraPerceptor.h"
#include "perception/impl/CameraPerceptor.h"

#include <chrono>
#include <opencv/cv.h>

using namespace taco;
using namespace cv;
using namespace std::chrono;

CameraDecoder::CameraDecoder(ICameraConfig::ConstPtr config, IEventLogger::ConstPtr logger)
	: Decoder(logger), _perceptorName(config->getPerceptorName()), _time(0), _frameWidth(config->getFrameWidth()),
	  _frameHeight(config->getFrameHeight())
{
	_pins.push_back(InputPin(config->getPerceptorName(), "VIDEO"));
}

CameraDecoder::~CameraDecoder()
{
}

std::vector<IPerceptor::ConstPtr> CameraDecoder::decode(
		const InputPin &pin, adtf::IMediaTypeDescription *mediaTypeDescription, adtf::IMediaSample *mediaSample)
{
	std::vector<IPerceptor::ConstPtr> perceptors;
	int pinIndex = indexOfPin(pin);

	if (pinIndex >= 0) {
		__adtf_sample_read_lock(mediaSample, void, l_pSrcBuffer);
		if (l_pSrcBuffer) {
			static int counter;
			Mat mat(Size(_frameWidth, _frameHeight), CV_8UC3, const_cast<void *>(l_pSrcBuffer));
			Mat img = mat.clone();

			// Update internal values
			counter++;
			_time = duration_cast<milliseconds>(high_resolution_clock::now().time_since_epoch()).count();
			// Create new CameraPerceptor
			perceptors.push_back(boost::make_shared<CameraPerceptor>(_perceptorName, _time, img));

		} else {
			LOG_INFO(adtf_util::cString::Format("could not lock depth image media sample"));
		}
	}

	return perceptors;
}
