#pragma once

#include "Angle.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <boost/smart_ptr.hpp>
#include <iostream>

namespace taco
{
class Line2D;
class Circle2D;
class Polygon;
class LineSegment;
class BoundingBox2D;
class AlignedBoundingBox3D;

/** \brief Representation of an pose (position and orientation) in 2D-space.
 *
 * The Pose2D class represents a pose (position and angle) in the 2-dimensional space.
 *
 * \author Stefan Glaser
 */
class Pose2D
{
  public:
	/** Default constructor creating a (0, 0, 0) pose. */
	Pose2D();

	/** Copy constructor, copying the position and angle from the other pose. */
	Pose2D(const Pose2D &other);

	/** Construct a new pose at the given position with zero angle. */
	Pose2D(const Eigen::Vector2d &position);

	/** Construct a new pose at the given x-, y-position with zero angle. */
	Pose2D(const double &x, const double &y);

	/** Construct a new pose at position (0, 0) with the given angle. */
	Pose2D(const Angle &angle);

	/** Construct a new pose at/with the given position/angle. */
	Pose2D(const Eigen::Vector2d &position, const Angle &angle);

	/** Construct a new pose at/with the given x-, y-position/angle. */
	Pose2D(const double &x, const double &y, const Angle &angle);

	~Pose2D();

	/** Retrieve the x-position value. */
	double x() const;

	/** Retrieve the y-position value. */
	double y() const;

	/** Retrieve the pose-position. */
	const Eigen::Vector2d &getPosition() const;

	/** Retrieve the pose-angle. */
	const Angle &getAngle() const;

	Pose2D &operator=(const Pose2D &rhs);

	/** Concatenate this pose with the given pose.
	 * Calculate the "virtual matrix product" of: this * rhs.
	 */
	Pose2D &operator*=(const Pose2D &rhs);

	/** Build the difference of two coordinate systems defines by the two poses, in the sense of: target - origin.
	 * This method calculates the pose X, such that the formula "origin + X = target" holds.
	 * Note that both poses (this pose and the rhs pose) must be defined with respect to the same base system
	 * for this operation to work as intended!
	 * Calculate the "virtual matrix product" of: rhs^-1 * this.
	 */
	Pose2D &operator-=(const Pose2D &rhs);

	/** Concatenate the inverse of this pose with the given pose.
	 * Calculate the "virtual matrix product" of: this^-1 * rhs.
	 */
	Pose2D &operator/=(const Pose2D &rhs);

	/** Set the position of this pose to the given position. */
	void setPosition(const Eigen::Vector2d &position);

	/** Set the position of this pose to the given x-, y-position values. */
	void setPosition(const double &x, const double &y);

	/** Set the angle of this pose to the given angle. */
	void setAngle(const Angle &angle);

	/** add pose to other */
	const Pose2D operator+(const Pose2D &other) const;

	/** Concatenation of this and rhs. */
	const Pose2D operator*(const Pose2D &other) const;

	/** Concatenation of rhs^-1 and this. */
	const Pose2D operator-(const Pose2D &other) const;

	/** Concatenation of this^-1 and rhs. */
	const Pose2D operator/(const Pose2D &other) const;

	/** Transform the given line about this pose.
	 * Calculate the "virtual matrix product" of: this * line
	 */
	const Line2D operator*(const Line2D &line) const;

	/** Inverse-transform the given line about this pose.
	 * Calculate the "virtual matrix product" of: this^-1 * line
	 */
	const Line2D operator/(const Line2D &line) const;

	/** Transform the given circle about this pose.
	 * Calculate the "virtual matrix product" of: this * origin
	 */
	const Circle2D operator*(const Circle2D &circle) const;

	/** Inverse-transform the circle about this pose.
	 * Calculate the "virtual matrix product" of: this^-1 * origin
	 */
	const Circle2D operator/(const Circle2D &circle) const;

	/** Transform the given line segment about this pose.
	 * Calculate the "virtual matrix product" of: this * line segment
	 */
	const LineSegment operator*(const LineSegment &lineSegment) const;

	/** Inverse-transform the line segment about this pose.
	 * Calculate the "virtual matrix product" of: this^-1 * line segment
	 */
	const LineSegment operator/(const LineSegment &lineSegment) const;

	/** Transform the given polygon points about this pose.
	 * Calculate the "virtual matrix product" of: this * points
	 */
	const boost::shared_ptr<Polygon> operator*(boost::shared_ptr<const Polygon> polygon) const;

	/** Inverse-transform the given polygon about this pose.
	 * Calculate the "virtual matrix product" of: this^-1 * points
	 */
	const boost::shared_ptr<Polygon> operator/(boost::shared_ptr<const Polygon> polygon) const;

	/** Transform the given vector about this pose.
	 * Calculate the "virtual matrix product" of: this * vec
	 */
	const Eigen::Vector2d operator*(const Eigen::Vector2d &vec) const;

	/** Inverse-transform the given vector about this pose.
	 * Calculate the "virtual matrix product" of: this^-1 * vec
	 */
	const Eigen::Vector2d operator/(const Eigen::Vector2d &vec) const;

	/** Transform the given 2d bounding box about this pose.
	 *
	 */
	const BoundingBox2D operator*(const BoundingBox2D &box) const;

	/** Inverse-transform the given 2d bounding box about this pose.
	 *
	 */
	const BoundingBox2D operator/(const BoundingBox2D &box) const;

	/** Transform the given 3d bounding box about this pose.
	 *  tanslating in x-y plane
	 */
	const AlignedBoundingBox3D operator*(const AlignedBoundingBox3D &box) const;

	/** Inverse-transform the given 3d bounding box about this pose.
	 *  tanslating in x-y plane
	 */
	const AlignedBoundingBox3D operator/(const AlignedBoundingBox3D &box) const;

	bool operator==(const Pose2D &other) const;
	bool operator!=(const Pose2D &other) const;

	/** Retrieve an inverse copy of this pose. */
	const Pose2D invert() const;

	/** Calculate the distance of this pose to the other pose. */
	const double getDistanceTo(const Pose2D &other) const;

	/** Returns the difference of this angle to the passed angle. */
	const Angle getDeltaAngle(const Pose2D &other) const;

	/** Returs a unit vector in direction of the angle of this pose. */
	const Eigen::Vector2d getUnitVector() const;

	/** Returns the angle to the position of the given pose relative to this pose. */
	const Angle getAngleTo(const Pose2D &other) const;

	/** Returns the angle to the given position relative to this pose. */
	const Angle getAngleTo(const Eigen::Vector2d &position) const;

	// ========== STATIC HELPER METHODS ==========
	static const Pose2D average(const Pose2D &p1, const Pose2D &p2, const double &weight1, const double &weight2);

	static const Pose2D average(const std::vector<Pose2D> &poses);

	static const Pose2D interpolate(const Pose2D &startPose, const Pose2D &endPose, const double &progress);

  private:
	/** The position of this pose. */
	Eigen::Vector2d _position;

	/** The angle of this pose. */
	Angle _angle;
};

std::ostream &operator<<(std::ostream &os, const Pose2D &pose);
}
