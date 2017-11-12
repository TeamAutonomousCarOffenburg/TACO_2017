#pragma once

#include "IManeuverGeometry.h"

#include <eigen3/Eigen/Dense>
#include <vector>

namespace taco
{
class Angle;
class Pose2D;

/** \brief Representation of a line in 2D-space.
 *
 * The Line2D class represents a line in 2-dimensional space, by describing a start point and an extension vector.
 * This way the Line2D class can represent both: a restricted line segment and the whole line in one class definition.
 *
 * \author Stefan Glaser
 */
class Line2D : public virtual IManeuverGeometry
{
  public:
	/** Construct a line from point (0, 0) to (1, 0) */
	Line2D();

	/** Construct a line with the given slope and y-offset
	 * \param m - the slope of the line
	 * \param b - the y-offset of the line
	 */
	Line2D(const double &m, const double &b);

	/** Construct a line with a start point, and angle and length of the line.
	 * \param xStart - the x-coordinate of the start point (or an arbitrary point on the line)
	 * \param yStart - the y-coordinate of the start point (or an arbitrary point on the line)
	 * \param alpha - the angle from the x-axis to the line
	 * \param length - the line segment length (or any value > 0)
	 */
	Line2D(const double &xStart, const double &yStart, const Angle &alpha, const double &length = 1);

	/** Construct a line with a start point, and angle and length of the line.
	 * \param start - the start point of the line segment (or an arbitrary point on the line)
	 * \param alpha - the angle from the x-axis to the line
	 * \param length - the line segment length (or any value > 0)
	 */
	Line2D(const Eigen::Vector2d &start, const Angle &alpha, const double &length = 1);

	/** Construct a line with a start and end point (or two arbitrary points on a the line).
	 * \param start - the start point of the line segment
	 * \param end - the end point of the line segment
	 */
	Line2D(const Eigen::Vector2d &start, const Eigen::Vector2d &end);

	/** Construct a line with a start and end point (or two arbitrary points on a the line).
	 * \param xStart - the x-coordinate of the start point of the line segment
	 * \param yStart - the y-coordinate of the start point of the line segment
	 * \param xEnd - the x-coordinate of the end point of the line segment
	 * \param yEnd - the y-coordinate of the end point of the line segment
	 */
	Line2D(const double &xStart, const double &yStart, const double &xEnd, const double &yEnd);
	~Line2D();

	/** Retrieve the start point of the line segment. */
	const Eigen::Vector2d &getStart() const;

	/** Retrieve the line segment extension vector. */
	const Eigen::Vector2d &getExtensionVector() const;

	/** Retrieve the end point of the line segment. */
	const Eigen::Vector2d getEnd() const;

	/** Retrieve the angle from the x-axis to this line. */
	const Angle getAngle() const;

	/** Retrieve the slope of this line (of line-formula: y = m*x + b). */
	const double m() const;

	/** Retrieve the y-offset of this line (of line-formula: y = m*x + b). */
	const double b() const;

	/** Retrieve the y-value of this line at the given x-value */
	const double yValue(const double &x) const;

	/** Retrieve the x-value of this line at the given y-value */
	const double xValue(const double &y) const;

	/** Retrieve the closest point and slope angle of this point on the line from given point */
	virtual Pose2D getClosestPose(const Eigen::Vector2d &point);

	/** Retrieve trail as vector of trail points */
	virtual std::vector<Eigen::Vector2d> getTrail(const Eigen::Vector2d &start, const Eigen::Vector2d &end);

	bool operator==(const Line2D &other) const;
	bool operator!=(const Line2D &other) const;

  private:
	/** Validate that this instance represents a proper line. If not, make this line the x-axis. */
	void validate();

	/** The start point of the line segment. */
	Eigen::Vector2d _start;

	/** The line segment extension. */
	Eigen::Vector2d _extension;
};
}
