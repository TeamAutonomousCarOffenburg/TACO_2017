#pragma once

#include "IManeuverGeometry.h"
#include <eigen3/Eigen/Dense>
#include <iostream>

namespace taco
{
class Line2D;
class Pose2D;

/** \brief Representation of a circle in 2D-space.
 *
 * The Circle2D class represents a circle in 2-dimensional space, by describing its origin and radius.
 *
 * \author Stefan Glaser, Peter Walden
 */
class Circle2D : public virtual IManeuverGeometry
{
  public:
	/** Construct a circle at (0, 0) with a radius of 1. */
	Circle2D();

	/** Construct a circle at (xOrigin, yOrigin) with the given radius.
	 * \param xOrigin - the x-coordinate of the center of the circle
	 * \param yOrigin - the y-coordinate of the center of the circle
	 * \param radius - the radius of the circle
	 */
	Circle2D(const double &xOrigin, const double &yOrigin, const double &radius);

	/** Construct a circle at the given origin and radius.
	 * \param origin - the center (origin) of the circle
	 * \param radius - the radius of the circle
	 */
	Circle2D(const Eigen::Vector2d &origin, const double &radius);

	/** Construct a circle at the given origin. The radius is determined by an arbirtrary point on the circle.
	 * \param origin - the center (origin) of the circle
	 * \param point - an arbitrary point on the circle
	 */
	Circle2D(const Eigen::Vector2d &origin, const Eigen::Vector2d &point);

	/** Construct a circle at the given origin. The radius is determined by an arbirtrary point on the circle.
	 * \param xOrigin - the x-coordinate of the center (origin) of the circle
	 * \param yOrigin - the y-coordinate of the center (origin) of the circle
	 * \param px - the x-coordinate of an arbitrary point on the circle
	 * \param py - the y-coordinate of an arbitrary point on the circle
	 */
	Circle2D(const double &xOrigin, const double &yOrigin, const double &px, const double &py);

	/** Construct a circle from given tangent an another point
	 * \param tangent - the tangent of the circle as a Pose2D object
	 * \param point - another point on the circle
	 */
	Circle2D(const Pose2D tangent, Eigen::Vector2d point);

	~Circle2D();

	/** Retrieve the circle origin. */
	const Eigen::Vector2d &origin() const;

	/** Retrieve the circle radius. */
	const double &radius() const;

	/** Check if this circle is touching the other circle as an inner lying circle. */
	const bool checkInnerTouching(const Circle2D &other) const;

	/** Check if this circle is touching the other circle as an outer lying circle. */
	const bool checkOuterTouching(const Circle2D &other) const;

	/** Check if this circle is completely inside the other circle without touching or intersecting it. */
	const bool checkInnerCircle(const Circle2D &other) const;

	/** Check if this circle intersects with the other circle in two points (no touching involved!). */
	const bool checkIntersect(const Circle2D &other) const;

	/** Retrieve the distance of this circle's origin to the other circle's origin. */
	const double getDistanceTo(const Circle2D &other) const;

	/** Retrieve the point on this circle at the given angle. */
	const Eigen::Vector2d getPointOnCircle(const Angle &angle) const;

	/** Retrieve Angle on circle no matter point is on circle */
	const Angle getAngle(const Eigen::Vector2d &point);

	/** Retrieve the Angle from given circular segment length, negative segment length =^ negativ angle
	 *  max segment length 1*PI
	 */
	const Angle getAngle(const double &segmentLength);

	/** Check if point is on circle */
	const bool isOnCircle(const Eigen::Vector2d &point) const;

	/** Retrieve the closest point and slope of this point on circle from given point */
	virtual Pose2D getClosestPose(const Eigen::Vector2d &point);

	/** Get new angle of slope on given angle */
	Angle slope(const Eigen::Vector2d &point);

	/** Retrieve direction of rotation (1 = counter clockwise, -1 = clockwise) from given point and angle of movement */
	short getDirectionOfRotation(const Eigen::Vector2d &point, const Angle &angle);
	short getDirectionOfRotation(const Pose2D pose);

	virtual std::vector<Eigen::Vector2d> getTrail(const Eigen::Vector2d &start, const Eigen::Vector2d &end);

	bool operator==(const Circle2D &other) const;
	bool operator!=(const Circle2D &other) const;

	static const Circle2D avg(
			const Circle2D &c1, const Circle2D &c2, const double &weight1 = 1, const double &weight2 = 1);

	static const Circle2D avg(const std::vector<Circle2D> &circles);

  private:
	/** A origin of the circle. */
	Eigen::Vector2d _origin;

	/** The radius of the circle. */
	double _radius;
};
}
