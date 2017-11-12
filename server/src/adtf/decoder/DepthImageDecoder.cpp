#include "DepthImageDecoder.h"
#include "perception/IPointCloudPerceptor.h"
#include "perception/impl/PointCloudPerceptor.h"

#include "utils/pcl/DepthImageToPointCloudConverter.h"

// #include <pcl/io/pcd_io.h>

#include <chrono>
#include <opencv2/imgproc/imgproc.hpp>

using namespace taco;
using namespace cv;
using namespace std::chrono;

DepthImageDecoder::DepthImageDecoder(ICameraConfig::ConstPtr config, IEventLogger::ConstPtr logger)
	: Decoder(logger), _perceptorName(config->getPerceptorName()), _time(0), _frameWidth(config->getFrameWidth()),
	  _frameHeight(config->getFrameHeight()), _focalLengthX(config->getFocalLengthX()),
	  _focalLengthY(config->getFocalLengthY())
{
	_pins.push_back(InputPin(config->getPerceptorName(), "VIDEO"));
}

DepthImageDecoder::~DepthImageDecoder()
{
}

std::vector<IPerceptor::ConstPtr> DepthImageDecoder::decode(
		const InputPin &pin, adtf::IMediaTypeDescription *mediaTypeDescription, adtf::IMediaSample *mediaSample)
{
	std::vector<IPerceptor::ConstPtr> perceptors;
	int pinIndex = indexOfPin(pin);

	if (pinIndex >= 0) {
		__adtf_sample_read_lock(mediaSample, void, l_pSrcBuffer);
		if (l_pSrcBuffer) {
			Mat *depthImage = new Mat(_frameHeight, _frameWidth, CV_16UC1, const_cast<void *>(l_pSrcBuffer));

			// Convert depth image to point cloud
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			cloud->height = _frameHeight;
			cloud->width = _frameWidth;
			DepthImageToPointCloudConverter::convert(depthImage, _focalLengthX, _focalLengthY, cloud);

			_time = duration_cast<milliseconds>(high_resolution_clock::now().time_since_epoch()).count();

			// save cloud as file
			// pcl::io::savePCDFileASCII ("cloud.pcd", *cloud);

			// Create new PointCloudPerceptor
			perceptors.push_back(boost::make_shared<PointCloudPerceptor>(_perceptorName, _time, cloud));

		} else {
			LOG_INFO(adtf_util::cString::Format("could not lock depth image media sample"));
		}
	}

	return perceptors;
}
