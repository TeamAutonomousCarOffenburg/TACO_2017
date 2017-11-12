#pragma once

#include "LaneDetectionConfiguration.h"
#include "LaneMiddle.h"
#include "PixelToCamera.h"
#include "ScanLine.h"
#include <opencv2/core/core.hpp>
#include <utils/geometry/Pose2D.h>
#include <utils/geometry/Pose3D.h>

namespace taco
{
class LaneAssist
{
  public:
	typedef boost::shared_ptr<LaneAssist> Ptr;
	typedef boost::shared_ptr<const LaneAssist> ConstPtr;

	LaneAssist(LaneDetectionConfiguration cfg);
	virtual ~LaneAssist();

	void detectLaneMiddle(cv::Mat &frame);

	void setPixelToCamera(PixelToCamera::ConstPtr pixelToCamera);

	LaneMiddle laneMiddle;

  private:
	LaneDetectionConfiguration _cfg;
	PixelToCamera::ConstPtr _pixelToCamera;

	void detection(cv::Mat &frame, LaneMiddle &result, int scanRightStartX);
	int reactiveDetectRight(cv::Mat &frame, LaneMiddle &resultMiddle, int scanLineStartX, int scanLineY);
	int reactiveDetectMiddle(cv::Mat &frame, LaneMiddle &resultMiddle, int y, int rightX);
	int reactiveDetectLeft(cv::Mat &frame, int y, int rightX, int middleX);
	void findMiddle(cv::Mat &frame, int &pixelX, int &pixelY, int scanX1, int scanX2, int y, int direction);
	void checkRange(int &value, int min, int max);
	void checkPlausibility(LaneMiddle &resultMiddle, int &pixelRightX, int &pixelMiddleX, int &pixelLeftX);
	void checkJump(int &pixelX, int oldX, int &delta);

	void prepareResult(const LaneMiddle &resultMiddle, int pixelRightX, int pixelMiddleX, int pixelLeftX, int &result,
			float &confidence) const;

	float getConfidenceFromDelta(float confidence, int delta1, int delta2) const;

	int findLineAhead(cv::Mat &frame, int x, int y);
};
}
