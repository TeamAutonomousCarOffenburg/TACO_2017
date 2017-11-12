#pragma once

#include "Pose2D.h"
#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Geometry>

namespace taco
{
/**
 *
 *
 * \author Peter Walden
 */

class BoundingBox2D
{
  public:
	typedef boost::shared_ptr<BoundingBox2D> Ptr;
	typedef boost::shared_ptr<const BoundingBox2D> ConstPtr;

	BoundingBox2D(const Pose2D &pose, const Eigen::Vector2d &minPoint, const Eigen::Vector2d &maxPoint);
	~BoundingBox2D();
	const Pose2D pose() const;
	const Eigen::Vector2d maxPoint() const;
	const Eigen::Vector2d minPoint() const;

  private:
	Pose2D _pose;
	Eigen::Vector2d _minPoint;
	Eigen::Vector2d _maxPoint;
};
}
