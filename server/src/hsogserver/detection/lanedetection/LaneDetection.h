#pragma once

#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>

#include "../ILaneDetection.h"
#include "LaneAssist.h"
#include "ScanLine.h"

#include "utils/concurrency/Worker.h"

namespace taco
{
class LaneDetection : public virtual taco::ILaneDetection, public taco::Worker
{
  public:
	LaneDetection(LaneDetectionConfiguration cfg);
	~LaneDetection();

	void detectLanes();
	void detectInitialAlignment();

	// worker
	void init();
	void start();
	void stop();
	void notifyPossibleNewFrame();
	void setData(cv::Mat &frame, const Eigen::Vector3d &upVector);

	taco::LaneAssist::Ptr getLaneResult();

  private:
	void run();
	LaneDetectionConfiguration _cfg;
	cv::Mat _frame;
	Eigen::Vector3d _upVector;
	std::mutex _updateFrame;
	std::mutex _resultLock;
	void setLaneResult(const taco::LaneAssist &result);
	taco::LaneAssist::Ptr _result;

	int _width;
	int _height;
	taco::LaneAssist detector;
};
}