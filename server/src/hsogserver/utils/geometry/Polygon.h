#pragma once

#include <boost/smart_ptr.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>

namespace taco
{
class Angle;
class Line2D;

/** \brief Representation of an polygon in 2D-space.
 *
 * The Polygon class represents a polygon in 2D space.
 * The polygon is defined by a list of clockwise sorted corner points.
 *
 * \author Stefan Glaser
 */
class Polygon
{
  public:
	typedef boost::shared_ptr<Polygon> Ptr;
	typedef boost::shared_ptr<const Polygon> ConstPtr;

	Polygon(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &p3);
	Polygon(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &p3, const Eigen::Vector2d &p4);
	Polygon(const std::vector<Eigen::Vector2d> &points);
	~Polygon();

	const Eigen::Vector2d &getCentroid() const;
	const double &getArea() const;
	const std::vector<Eigen::Vector2d> &getPoints() const;

	const bool isInside(const Eigen::Vector2d &point) const;
	const double getDistanceTo(const Eigen::Vector2d &point) const;
	const Line2D getClosestPolyLine(const Eigen::Vector2d &point) const;
	const double getAvgDistance(Polygon *other) const;
	const std::pair<Eigen::Vector2d, Eigen::Vector2d> getBounds() const;

  private:
	void checkArea();

	/** The centroid of the polygon. */
	Eigen::Vector2d _centroid;

	/** The area of the polygon. */
	double _area;

	/** The polygon corner points (in clockwise order). */
	std::vector<Eigen::Vector2d> _points;
};
}
