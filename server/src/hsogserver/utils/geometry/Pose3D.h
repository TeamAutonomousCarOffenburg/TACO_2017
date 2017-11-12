#pragma once

#include "../hsogserver/utils/geometry/AlignedBoundingBox3D.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>

namespace taco
{
/** \brief Representation of an pose (position and orientation) in 3D-space.
 *
 * The Pose3D class represents a pose (position and orientation) in the 3-dimensional space.
 *
 * \author Stefan Glaser
 */
class Pose3D
{
  public:
	/** Default constructor creating a pose at position (0, 0, 0) with identity orientation. */
	Pose3D();

	/** Copy constructor, copying the position and angle from the other pose. */
	Pose3D(const Pose3D &other);

	/** Construct a new pose at the given position with identity orientation. */
	Pose3D(const Eigen::Vector3d &position);

	/** Construct a new pose at the given x-, y-, z-position with identity orientation. */
	Pose3D(const double &x, const double &y, const double &z);

	/** Construct a new pose at position (0, 0, 0) with the given orientation. */
	Pose3D(const Eigen::Quaterniond &orientation);

	/** Construct a new pose at/with the given position/orientation. */
	Pose3D(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation);

	/** Construct a new pose at/with the given x-, y-, z-position/orientation. */
	Pose3D(const double &x, const double &y, const double &z, const Eigen::Quaterniond &orientation);

	/** Construct a new pose at position (0, 0, 0) with the given orientation. */
	Pose3D(const Eigen::AngleAxisd &orientation);

	/** Construct a new pose at/with the given position/orientation. */
	Pose3D(const Eigen::Vector3d &position, const Eigen::AngleAxisd &orientation);

	/** Construct a new pose at/with the given x-, y-, z-position/orientation. */
	Pose3D(const double &x, const double &y, const double &z, const Eigen::AngleAxisd &orientation);
	~Pose3D();

	/** Retrieve the x-position value. */
	double x() const;

	/** Retrieve the y-position value. */
	double y() const;

	/** Retrieve the z-position value. */
	double z() const;

	/** Retrieve the pose-position. */
	const Eigen::Vector3d &getPosition() const;

	/** Retrieve the pose-orientation. */
	const Eigen::Quaterniond &getOrientation() const;

	Pose3D &operator=(const Pose3D &rhs);

	/** Concatenate this pose with the given pose.
	 * Calculate the "virtual matrix product" of: this * rhs.
	 */
	Pose3D &operator*=(const Pose3D &rhs);

	/** Build the difference of two coordinate systems defines by the two poses, in the sense of: target - origin.
	 * This method calculates the pose X, such that the formula "origin + X = target" holds.
	 * Note that both poses (this pose and the rhs pose) must be defined with respect to the same base system
	 * for this operation to work as intended!
	 * Calculate the "virtual matrix product" of: rhs^-1 * this.
	 */
	Pose3D &operator-=(const Pose3D &rhs);

	/** Concatenate the inverse of this pose with the given pose.
	 * Calculate the "virtual matrix product" of: this^-1 * rhs.
	 */
	Pose3D &operator/=(const Pose3D &rhs);

	/** Set the position of this pose to the given position. */
	void setPosition(const Eigen::Vector3d &position);

	/** Set the position of this pose to the given x-, y-, z-position values. */
	void setPosition(const double &x, const double &y, const double &z);

	/** Set the angle of this pose to the given orientation. */
	void setOrientation(const Eigen::Quaterniond &orientation);

	/** Set the angle of this pose to the given orientation. */
	void setOrientation(const Eigen::AngleAxisd &orientation);

	/** Concatenation of this and rhs. */
	const Pose3D operator*(const Pose3D &other) const;

	/** Concatenation of rhs^-1 and this. */
	const Pose3D operator-(const Pose3D &other) const;

	/** Concatenation of this^-1 and rhs. */
	const Pose3D operator/(const Pose3D &other) const;

	/** Transform the given vector about this pose.
	 * Calculate the "virtual matrix product" of: this * vec
	 */
	const Eigen::Vector3d operator*(const Eigen::Vector3d &vec) const;

	/** Inverse-transform the given vector about this pose.
	 * Calculate the "virtual matrix product" of: this^-1 * vec
	 */
	const Eigen::Vector3d operator/(const Eigen::Vector3d &vec) const;

	/** Transform the given 3d bounding box about this pose.
	 *
	 */

	const AlignedBoundingBox3D operator*(const AlignedBoundingBox3D &box) const;

	/** Inverse-transform the given 3d bounding box about this pose.
	 *
	 */
	const AlignedBoundingBox3D operator/(const AlignedBoundingBox3D &box) const;

	bool operator==(const Pose3D &other) const;
	bool operator!=(const Pose3D &other) const;

	/** Retrieve an inverse copy of this pose. */
	const Pose3D invert() const;

  private:
	/** The position of this pose. */
	Eigen::Vector3d _position;

	/** The orientation of this pose. */
	Eigen::Quaterniond _orientation;
};

std::ostream &operator<<(std::ostream &os, const Pose3D &pose);
}
