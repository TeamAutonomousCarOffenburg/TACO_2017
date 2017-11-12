#include "Line2D.h"
#include "Angle.h"
#include "Geometry.h"
#include "Pose2D.h"

#include <boost/concept_check.hpp>
#include <cmath>
#include <limits>

using namespace taco;
using namespace Eigen;

Line2D::Line2D() : _start(0, 0), _extension(1, 0)
{
}

Line2D::Line2D(const double &m, const double &b) : _start(0, b), _extension(1, m)
{
}

Line2D::Line2D(const double &xStart, const double &yStart, const Angle &alpha, const double &length)
	: _start(xStart, yStart), _extension(alpha.getVector(length))
{
	if (_extension(0) == 0 && _extension(1) == 0) {
		_extension = Vector2d(1, 0);
	}
}

Line2D::Line2D(const Eigen::Vector2d &start, const Angle &alpha, const double &length)
	: _start(start), _extension(alpha.getVector(length))
{
	validate();
}

Line2D::Line2D(const Vector2d &start, const Vector2d &end) : _start(start), _extension(end - start)
{
	validate();
}

Line2D::Line2D(const double &xStart, const double &yStart, const double &xEnd, const double &yEnd)
	: _start(xStart, yStart), _extension(xEnd - xStart, yEnd - yStart)
{
	validate();
}

Line2D::~Line2D()
{
}

void Line2D::validate()
{
	if (_extension(0) == 0 && _extension(1) == 0) {
		_extension = Vector2d(0, 1);
	}
}

const Eigen::Vector2d &Line2D::getStart() const
{
	return _start;
}

const Eigen::Vector2d &Line2D::getExtensionVector() const
{
	return _extension;
}

const Eigen::Vector2d Line2D::getEnd() const
{
	return _start + _extension;
}

const Angle Line2D::getAngle() const
{
	return Angle::to(_extension);
}

const double Line2D::m() const
{
	if (_extension(0) == 0) {
		if (_extension(1) > 0) {
			return std::numeric_limits<double>::max();
		} else {
			return std::numeric_limits<double>::lowest();
		}
	}

	return _extension(1) / _extension(0);
}

const double Line2D::b() const
{
	if (_extension(0) == 0) {
		return 0;
	}

	return _start(1) - ((_extension(1) / _extension(0)) * _start(0));
}

const double Line2D::yValue(const double &x) const
{
	if (_extension(0) == 0) {
		// this line is y-parallel
		return 0;
	}

	return (_extension(1) / _extension(0)) * (x - _start(0)) + _start(1);
}

const double Line2D::xValue(const double &y) const
{
	if (_extension(1) == 0) {
		// this line is x-parallel
		return _start(0);
	}

	return (_extension(0) / _extension(1)) * (y - _start(1)) + _start(0);
}

Pose2D Line2D::getClosestPose(const Vector2d &point)
{
	Angle lineAngle(Angle::to(_extension));
	Angle pointAngle(Angle::to(point - _start));

	Angle offsetAngle(pointAngle - lineAngle);
	double hypotenuse = Geometry::getDistance<double, 2>(point, _start);
	double factor = cos(offsetAngle.rad()) * hypotenuse;
	Vector2d closestPoint(_start + _extension / Geometry::getNorm<double, 2>(_extension) * factor);

	return Pose2D(closestPoint, Angle::to(_extension));
}

std::vector<Vector2d> Line2D::getTrail(const Vector2d &start, const Vector2d &end)
{
	Pose2D poseStart = getClosestPose(start);
	Pose2D poseEnd = getClosestPose(end);
	double distance = poseStart.getDistanceTo(poseEnd);
	Vector2d offset = poseStart.getAngle().getVector(0.1 * distance);
	Vector2d point = poseStart.getPosition();
	std::vector<Vector2d> points;
	int i = 0;
	do {
		point += offset;
		points.push_back(point);
	} while (++i < 10);
	return points;
}

bool Line2D::operator==(const Line2D &other) const
{
	return _start == other._start && _extension == other._extension;
}

bool Line2D::operator!=(const Line2D &other) const
{
	return !(*this == other);
}
