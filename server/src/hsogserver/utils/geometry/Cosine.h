#pragma once
#include "utils/geometry/IManeuverGeometry.h"
#include "utils/geometry/Pose2D.h"

namespace taco
{
/** \brief Representation of a cosine in 2-dimensional space.
 *
 * The Cosine class represents a cosine in 2-dimensional space, by describing its origin, elongation and spreading.
 *
 * \author Peter Walden
 */
class Cosine : public virtual IManeuverGeometry
{
  public:
	Cosine();

	Cosine(const Pose2D &origin, const double &elongation, const double &spreading);
	~Cosine();
	virtual Pose2D getClosestPose(const Eigen::Vector2d &point);

	Pose2D &origin();
	double &elongation();
	double &spreading();
	virtual std::vector<Eigen::Vector2d> getTrail(const Eigen::Vector2d &start, const Eigen::Vector2d &end);
	//** normalize the ratio from elongation / spreading to 1 by adjusting the elongation around given point  */
	void normalizeRatioToPoint(Eigen::Vector2d point);
	/** Validate that this instance represents a proper cosine. If not, make this cosine the standard cosine */
	void validate();

  private:
	Pose2D _origin;
	double _elongation;
	double _spreading;
};
}