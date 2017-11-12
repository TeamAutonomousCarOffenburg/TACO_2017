#pragma once

#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Geometry>

namespace taco
{
/**
 *
 *
 * \author Peter Walden
 */

class AlignedBoundingBox3D
{
  public:
	typedef boost::shared_ptr<AlignedBoundingBox3D> Ptr;
	typedef boost::shared_ptr<const AlignedBoundingBox3D> ConstPtr;

	AlignedBoundingBox3D(
			const Eigen::Vector3d &origin, const Eigen::Vector3d &minPoint, const Eigen::Vector3d &maxPoint);
	AlignedBoundingBox3D(const Eigen::Vector3d &origin, const Eigen::Vector3d &minPoint,
			const Eigen::Vector3d &maxPoint, uint score);
	~AlignedBoundingBox3D();

	const Eigen::Vector3d origin() const;

	const Eigen::Vector3d maxPoint() const;

	const Eigen::Vector3d minPoint() const;

	const Eigen::Vector3d midpoint() const;

	void setScoreLimit(const uint &limit);

	const uint score() const;

	const AlignedBoundingBox3D operator+(const AlignedBoundingBox3D &other) const;

	const uint operator+(const uint &other) const;

	const uint operator-(const uint &other) const;

	const void operator+=(const uint &other);

	const void operator-=(const uint &other);

	const AlignedBoundingBox3D &operator++();

	const AlignedBoundingBox3D &operator--();

	//* has intersection */
	bool operator==(const AlignedBoundingBox3D &other) const;

	bool operator<(const uint &other) const;

	bool operator>(const uint &other) const;

	bool operator<=(const uint &other) const;

	bool operator>=(const uint &other) const;

	bool operator==(const uint &other) const;

	bool operator!=(const uint &other) const;

  private:
	uint _scoreLimit = 10;
	Eigen::Vector3d _origin;
	Eigen::Vector3d _minPoint;
	Eigen::Vector3d _maxPoint;
	uint _score = 0;
};
}
