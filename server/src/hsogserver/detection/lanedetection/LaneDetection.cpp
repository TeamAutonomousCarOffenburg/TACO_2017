#include "LaneDetection.h"
#include <opencv2/imgproc.hpp>

#include "EdgeDetector.h"
#include "LaneDetectionConfiguration.h"
#include "PixelToCamera.h"

using namespace taco;

LaneDetection::LaneDetection(LaneDetectionConfiguration cfg) : _cfg(cfg), detector(cfg)
{
	_height = cfg.height;
	_width = cfg.width;
}

LaneDetection::~LaneDetection()
{
}

void LaneDetection::run()
{
	while (!m_stop) {
		detectLanes();
	}
}

void LaneDetection::setData(cv::Mat &frame, const Eigen::Vector3d &upVector)
{
	_updateFrame.lock();
	_frame = frame;
	_upVector = upVector;
	_updateFrame.unlock();
}

void LaneDetection::start()
{
	Runnable::start("LaneDetection");
}

void LaneDetection::stop()
{
	Runnable::stop();
}

void LaneDetection::notifyPossibleNewFrame()
{
	Worker::notify();
}

// for use with model
void LaneDetection::detectLanes()
{
	static void *oldFrameDataLocation = nullptr;

	if (!wait2()) {
		return;
	}

	// ------------------------------------------------------------
	// Fetch data synchronized
	_updateFrame.lock();
	cv::Mat frame(_frame);
	Eigen::Vector3d upVector = _upVector;
	_updateFrame.unlock();
	// ------------------------------------------------------------

	// Check if we received a new frame
	if (frame.empty() || frame.data == oldFrameDataLocation) {
		return;
	}

	oldFrameDataLocation = frame.data;

	// image conversions and edge detection
	cv::Mat edges = EdgeDetector::detectEdges(frame, _cfg.width, _cfg.height * 0.5);
	PixelToCamera::ConstPtr pixelToCam = boost::make_shared<PixelToCamera>(
			upVector, _cfg.focalVertical, _cfg.focalHorizontal, _cfg.focalPointX, _cfg.focalPointY - _cfg.height * 0.5);
	// TODO: integrate pixelToCamera class into LaneAssist
	detector.setPixelToCamera(pixelToCam);

	detector.detectLaneMiddle(edges);

	setLaneResult(detector);
}

void LaneDetection::setLaneResult(const LaneAssist &result)
{
	_resultLock.lock();
	_result = boost::make_shared<LaneAssist>(result);
	_resultLock.unlock();
}

LaneAssist::Ptr LaneDetection::getLaneResult()
{
	_resultLock.lock();
	LaneAssist::Ptr res = _result;
	_result = LaneAssist::Ptr();
	_resultLock.unlock();
	return res;
}
