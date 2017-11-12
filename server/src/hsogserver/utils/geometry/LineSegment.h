#pragma once

#include "Pose2D.h"

#include <iostream>
#include <vector>

namespace taco
{
class Circle2D;

/** \brief Representation of an circular arc in 2D-space.
 *
 * The LineSegment class represents an circular arc in 2-dimensional space.
 * The arc is specified by an length and exit angle and shifted to a start pose.
 *
 * \author Stefan Glaser
 */
class LineSegment
{
  public:
	LineSegment(const Pose2D &startPose, const double &length, const double &exitAngle);
	LineSegment(const Circle2D &circle, const Eigen::Vector2d &startPos, const Eigen::Vector2d &endPos,
			const bool &clockwise);
	LineSegment(const Circle2D &circle, const Angle &startAngle, const Angle &endAngle, const bool &clockwise);
	~LineSegment();

	const Pose2D &getStartPose() const;

	const Pose2D &getEndPose() const;

	const double &getLength() const;

	const double &getExitAngle() const;

	const double getRadius() const;

	const Pose2D interpolatePose(const double &length) const;

	void interpolateIntermediatePoses(const size_t &howMany, std::vector<Pose2D> &result) const;

	const bool isOnSegment(const Eigen::Vector2d &position) const;
	const double getExtension(const Eigen::Vector2d &position) const;
	const double getDistanceTo(const Eigen::Vector2d &position) const;

  private:
	/** The start pose of the line segment. */
	Pose2D _startPose;

	/** The end pose of the line segment. */
	Pose2D _endPose;

	/** The length of the line segment. */
	double _length;

	/** The exit angle of the line segment in the range [-2 PI, 2 PI]. */
	double _exitAngle;
};

std::ostream &operator<<(std::ostream &os, const taco::LineSegment &segment);
}
