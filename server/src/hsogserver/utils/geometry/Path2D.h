#pragma once

#include "LineSegment.h"
#include "Pose2D.h"

#include <boost/smart_ptr.hpp>
#include <vector>

namespace taco
{
/** \brief Representation of an path (trajectory) in 2D-space.
 *
 * The Path class represents an path (trajectory in 2-dimensional space.
 * The path is constructed by a list of way points, that are connected with line segments.
 *
 * \author Stefan Glaser
 */
class Path2D
{
  public:
	typedef boost::shared_ptr<Path2D> Ptr;
	typedef boost::shared_ptr<const Path2D> ConstPtr;

	Path2D();
	~Path2D();

	const double &getLength() const;
	const Pose2D &getStartPose() const;
	const Pose2D &getEndPose() const;
	const std::vector<Pose2D> &getWayPoints() const;
	const Pose2D getLastWayPoint() const;
	const std::vector<LineSegment> &getLineSegments() const;

	void addWayPoint(const Pose2D &wayPoint);

	const double getPositionOnPath(const Eigen::Vector2d &position) const;
	const Pose2D interpolateWayPose(const double &length) const;

  private:
	/** The length of the path. */
	double _length;

	/** The start pose of the path. */
	Pose2D _startPose;

	/** The end pose of the path. */
	Pose2D _endPose;

	/** The list of way points this path is based on. */
	std::vector<Pose2D> _wayPoints;

	/** The list of line segments along the way points. */
	std::vector<LineSegment> _segments;
};
}