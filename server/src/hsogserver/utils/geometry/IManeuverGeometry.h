#pragma once

#include "Pose2D.h"
#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Dense>

namespace taco
{
/** \brief Interface: Class implementing this interface can be used by maneuvers
 *
 *
 *
 * \author Peter Walden
 */
class IManeuverGeometry
{
  public:
	typedef boost::shared_ptr<IManeuverGeometry> Ptr;
	typedef boost::shared_ptr<const IManeuverGeometry> ConstPtr;

	/** Retrieve the closest point and its slope on geometry from give point   */
	virtual Pose2D getClosestPose(const Eigen::Vector2d &point) = 0;
	/** Retrieve the path of the geometry from start to end   */
	virtual std::vector<Eigen::Vector2d> getTrail(const Eigen::Vector2d &start, const Eigen::Vector2d &end) = 0;
};
}
