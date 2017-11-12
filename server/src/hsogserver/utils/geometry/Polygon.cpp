#include "Polygon.h"
#include "Angle.h"
#include "Geometry.h"
#include "Line2D.h"

#include <algorithm>

using namespace taco;
using namespace Eigen;

Polygon::Polygon(const Vector2d &p1, const Vector2d &p2, const Vector2d &p3)
{
	_points.push_back(p1);
	_points.push_back(p2);
	_points.push_back(p3);
	_centroid = Geometry::avg(_points);
	_area = Geometry::getArea(_points);
	checkArea();
}

Polygon::Polygon(const Vector2d &p1, const Vector2d &p2, const Vector2d &p3, const Vector2d &p4)
{
	_points.push_back(p1);
	_points.push_back(p2);
	_points.push_back(p3);
	_points.push_back(p4);
	_centroid = Geometry::avg(_points);
	_area = Geometry::getArea(_points);
	checkArea();
}

Polygon::Polygon(const std::vector<Vector2d> &points)
{
	_points = points;
	_centroid = Geometry::avg(_points);
	_area = Geometry::getArea(_points);
	checkArea();
}

Polygon::~Polygon()
{
}

const Vector2d &Polygon::getCentroid() const
{
	return _centroid;
}

const double &Polygon::getArea() const
{
	return _area;
}

const std::vector<Vector2d> &Polygon::getPoints() const
{
	return _points;
}

const bool Polygon::isInside(const Vector2d &point) const
{
	return Geometry::isInsidePolygon(_points, point);
}

const double Polygon::getDistanceTo(const Vector2d &point) const
{
	return Geometry::getDistance(_points, point);
}

const Line2D Polygon::getClosestPolyLine(const Vector2d &point) const
{
	return Geometry::getClosestPolyLine(_points, point);
}

const double Polygon::getAvgDistance(Polygon *other) const
{
	return Geometry::getAvgDistance(_points, other->getPoints());
}

void Polygon::checkArea()
{
	if (_area < 0) {
		// Counter clockwise order, thus flip elements
		_area = -_area;
		std::reverse(_points.begin(), _points.end());
	}
}
