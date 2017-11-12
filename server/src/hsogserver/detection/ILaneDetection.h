#pragma once

#include <boost/smart_ptr.hpp>
#include <opencv2/core/core.hpp>

#include <eigen3/Eigen/Dense>

#include "detection/lanedetection/LaneAssist.h"

namespace taco
{
/**
 * Interface for the lane detection.
 *
 * \author Simon Danner
 */
class ILaneDetection
{
  public:
	virtual ~ILaneDetection(){};

	/** Start asynchroneous worker thread. */
	virtual void start() = 0;

	/** Stop asynchroneous worker thread. */
	virtual void stop() = 0;

	virtual void setData(cv::Mat &frame, const Eigen::Vector3d &upVector) = 0;
	virtual void notifyPossibleNewFrame() = 0;

	virtual void detectLanes() = 0;
	virtual LaneAssist::Ptr getLaneResult() = 0;

	typedef boost::shared_ptr<ILaneDetection> Ptr;
	typedef boost::shared_ptr<const ILaneDetection> ConstPtr;
};
}
