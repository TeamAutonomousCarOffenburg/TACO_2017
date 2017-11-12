#include "Circle2D.h"
#include "Geometry.h"
#include "Line2D.h"
#include "Pose2D.h"

#include <boost/concept_check.hpp>
#include <cmath>

using namespace taco;
using namespace Eigen;

Circle2D::Circle2D() : _origin(0, 0), _radius(1)
{
}

Circle2D::Circle2D(const double &xOrigin, const double &yOrigin, const double &radius)
	: _origin(xOrigin, yOrigin), _radius(radius >= 0 ? radius : -radius)
{
}

Circle2D::Circle2D(const Vector2d &origin, const double &radius) : Circle2D(origin(0), origin(1), radius)
{
}

Circle2D::Circle2D(const Vector2d &origin, const Vector2d &point)
	: _origin(origin), _radius(Geometry::getDistance(origin, point))
{
}

Circle2D::Circle2D(const double &xOrigin, const double &yOrigin, const double &px, const double &py)
	: _origin(xOrigin, yOrigin), _radius(Geometry::getDistance(xOrigin, yOrigin, px, py))
{
}

Circle2D::Circle2D(const Pose2D tangent, Vector2d point)
{
	// transform point in to an tangent touching point local coordinate system (tangent angle = x-axis + pi)
	Eigen::Vector2d tmpVector = point - tangent.getPosition();
	Eigen::Vector2d rotatedVector(0, 0);
	Angle::rotate(tmpVector, -tangent.getAngle().rad(), rotatedVector);
	// calculate y-coordinate of the circle origin
	double my = (-pow(rotatedVector(0), 2) - pow(rotatedVector(1), 2)) / (-2 * rotatedVector(1));
	// transform back to world coordinate system
	Eigen::Vector2d shiftedCircleOrigin(0, 0);
	Angle::rotate(Eigen::Vector2d(0, my), tangent.getAngle().rad(), shiftedCircleOrigin);
	_origin = shiftedCircleOrigin + tangent.getPosition();
	_radius = Geometry::getDistance(_origin, tangent.getPosition());
}

Circle2D::~Circle2D()
{
}

const Vector2d &Circle2D::origin() const
{
	return _origin;
}

const double &Circle2D::radius() const
{
	return _radius;
}

const bool Circle2D::checkInnerTouching(const Circle2D &other) const
{
	double distance = getDistanceTo(other);

	return distance > 0 && (_radius == distance + other._radius || other._radius == distance + _radius);
}

const bool Circle2D::checkOuterTouching(const Circle2D &other) const
{
	double distance = getDistanceTo(other);

	return _radius + other._radius == distance;
}

const bool Circle2D::checkIntersect(const Circle2D &other) const
{
	double distance = getDistanceTo(other);

	return distance > 0 && distance < _radius + other._radius && _radius < distance + other._radius &&
		   other._radius < distance + _radius;
}

const bool Circle2D::checkInnerCircle(const Circle2D &other) const
{
	double distance = getDistanceTo(other);

	return _radius > distance + other._radius || other._radius > distance + _radius;
}

const double Circle2D::getDistanceTo(const Circle2D &other) const
{
	return Geometry::getDistance(_origin, other._origin);
}

const Vector2d Circle2D::getPointOnCircle(const Angle &angle) const
{
	return Vector2d(_origin(0) + (std::cos(angle.rad()) * _radius), _origin(1) + (std::sin(angle.rad()) * _radius));
}

const bool Circle2D::isOnCircle(const Vector2d &point) const
{
	double distance = Geometry::getDistance(_origin, point);
	double diff = std::abs(distance - _radius);
	return (diff < 0.000001);
}

const Angle Circle2D::getAngle(const Vector2d &point)
{
	Vector2d shiftedPoint = point - _origin;
	// point is equal to origin
	if (std::fabs(point(0) - _origin(0)) < 0.001 && std::fabs(point(1) - _origin(1)) < 0.001)
		return Angle(0);
	// threshold x = 0
	if (std::fabs<double>(shiftedPoint(0)) < 0.0001) {
		if (shiftedPoint(1) > 0)
			return Angle(M_PI * 0.5);
		else
			return Angle(-M_PI * 0.5);
	}

	double angle = atan2(shiftedPoint(1), shiftedPoint(0));
	return Angle(angle);
}

const Angle Circle2D::getAngle(const double &segmentLength)
{
	return Angle(segmentLength / _radius);
}

Pose2D Circle2D::getClosestPose(const Vector2d &point)
{
	Angle angle = Angle::to(point - _origin);
	Vector2d closestPoint = getPointOnCircle(angle);
	return Pose2D(closestPoint, slope(closestPoint));
}

Angle Circle2D::slope(const Vector2d &point)
{
	// calculate slope on upper semicircle
	double slope = -0.5 * sqrt(1 / (pow(_radius, 2) - pow(point(0) - _origin(0), 2))) * (2 * point(0) - 2 * _origin(0));
	// check if point is on lower semicircle
	if (_origin(1) > point(1)) {
		slope *= -1;
	}
	return Angle(atan(slope));
}

short Circle2D::getDirectionOfRotation(const Vector2d &point, const Angle &angle)
{
	Angle angle2 = getAngle(point);
	angle2 += Angle(M_PI * 0.5);
	double angleResult = std::fabs<double>(angle2.rad() - angle.rad()) + M_PI * 0.5;

	if (angleResult < M_PI || angleResult > 2 * M_PI)
		return 1;
	else
		return -1;
}

short int Circle2D::getDirectionOfRotation(const Pose2D pose)
{
	return getDirectionOfRotation(pose.getPosition(), pose.getAngle());
}

std::vector<Vector2d> Circle2D::getTrail(const Vector2d &start, const Vector2d &end)
{
	Angle startAngle = getAngle(start);
	Angle endAngle = getAngle(end);
	Angle diff = endAngle - startAngle;
	Angle delta = Angle(diff.rad() / 10);
	Vector2d point;
	std::vector<Vector2d> points;
	int i = 0;
	do {
		startAngle += delta;
		point = getPointOnCircle(startAngle);
		points.push_back(point);
	} while (++i < 10);
	return points;
}

bool Circle2D::operator==(const Circle2D &other) const
{
	return _origin == other._origin && _radius == other._radius;
}

bool Circle2D::operator!=(const Circle2D &other) const
{
	return !(*this == other);
}

const Circle2D Circle2D::avg(const Circle2D &c1, const Circle2D &c2, const double &weight1, const double &weight2)
{
	Vector2d avgOrigin = Geometry::avg(c1._origin, c2._origin, weight1, weight2);

	return Circle2D(avgOrigin, c1._radius * weight1 + c2._radius * weight2);
}

const Circle2D Circle2D::avg(const std::vector<Circle2D> &circles)
{
	size_t size = circles.size();
	if (size == 0) {
		return Circle2D(0, 0, 1);
	} else if (size == 1) {
		return circles[0];
	}

	double x = 0;
	double y = 0;
	double radius = 0;

	for (Circle2D circle : circles) {
		x += circle._origin(0);
		y += circle._origin(1);
		radius += circle._radius;
	}

	return Circle2D(x / size, y / size, radius / size);
}
