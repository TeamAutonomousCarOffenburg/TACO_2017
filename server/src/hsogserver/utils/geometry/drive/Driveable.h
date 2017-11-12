#pragma once

#include "utils/geometry/IManeuverGeometry.h"
#include <boost/smart_ptr.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>

namespace taco
{
/** Class representing driveable path.
 *
 * \author Peter Walden
 */
class Driveable : public virtual IManeuverGeometry
{
  public:
	Driveable();
	Driveable(IManeuverGeometry::Ptr geometry);
	Driveable(IManeuverGeometry::Ptr geometry1, IManeuverGeometry::Ptr geometry2, const Pose2D &intercept);
	~Driveable();
	virtual Pose2D getClosestPose(const Eigen::Vector2d &point);
	virtual std::vector<Eigen::Vector2d> getTrail(const Eigen::Vector2d &start, const Eigen::Vector2d &end);
	bool isValid();
	static const Driveable &Invalid();

  private:
	static Driveable INVALID;
	std::vector<IManeuverGeometry::Ptr> _geometries;
	Pose2D _intercept;
	bool _valid = false;
};
}