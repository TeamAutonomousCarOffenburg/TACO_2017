#include "Pose2D.h"

#include "AlignedBoundingBox3D.h"
#include "BoundingBox2D.h"
#include "Circle2D.h"
#include "Line2D.h"
#include "LineSegment.h"
#include "Polygon.h"
#include <boost/concept_check.hpp>

using namespace taco;
using namespace Eigen;

Pose2D::Pose2D() : _position(0, 0), _angle(0)
{
}

Pose2D::Pose2D(const Pose2D &other) : _position(other._position), _angle(other._angle)
{
}

Pose2D::Pose2D(const Vector2d &position) : _position(position), _angle(0)
{
}

Pose2D::Pose2D(const double &x, const double &y) : _position(x, y), _angle()
{
}

Pose2D::Pose2D(const Angle &angle) : _position(0, 0), _angle(angle)
{
}

Pose2D::Pose2D(const Vector2d &position, const Angle &angle) : _position(position), _angle(angle)
{
}

Pose2D::Pose2D(const double &x, const double &y, const Angle &angle) : _position(x, y), _angle(angle)
{
}

Pose2D::~Pose2D()
{
}

double Pose2D::x() const
{
	return _position(0);
}

double Pose2D::y() const
{
	return _position(1);
}

const Vector2d &Pose2D::getPosition() const
{
	return _position;
}

const Angle &Pose2D::getAngle() const
{
	return _angle;
}

Pose2D &Pose2D::operator=(const Pose2D &rhs)
{
	_position = rhs._position;
	_angle = rhs._angle;
	return *this;
}

Pose2D &Pose2D::operator*=(const Pose2D &rhs)
{
	_position += _angle * rhs._position;
	_angle += rhs._angle;
	return *this;
}

Pose2D &Pose2D::operator-=(const Pose2D &rhs)
{
	_position = rhs._angle / (_position - rhs._position);
	_angle -= rhs._angle;
	return *this;
}

Pose2D &Pose2D::operator/=(const Pose2D &rhs)
{
	_position = _angle / (rhs._position - _position);
	_angle = rhs._angle - _angle;
	return *this;
}

void Pose2D::setPosition(const Vector2d &position)
{
	_position = position;
}

void Pose2D::setPosition(const double &x, const double &y)
{
	_position(0) = x;
	_position(1) = y;
}

void Pose2D::setAngle(const Angle &angle)
{
	_angle = angle;
}

const Pose2D Pose2D::operator+(const Pose2D &other) const
{
	return Pose2D(getPosition() + other.getPosition(), getAngle() + other.getAngle());
}

const Pose2D Pose2D::operator*(const Pose2D &other) const
{
	return Pose2D(*this) *= other;
}

const Vector2d Pose2D::operator*(const Vector2d &vec) const
{
	return _position + _angle * vec;
}

const Line2D Pose2D::operator*(const Line2D &line) const
{
	return Line2D(*this * line.getStart(), *this * line.getEnd());
}

const Line2D Pose2D::operator/(const Line2D &line) const
{
	return Line2D(*this / line.getStart(), *this / line.getEnd());
}

const Circle2D Pose2D::operator*(const Circle2D &circle) const
{
	return Circle2D(*this * circle.origin(), circle.radius());
}

const Circle2D Pose2D::operator/(const Circle2D &circle) const
{
	return Circle2D(*this / circle.origin(), circle.radius());
}

const LineSegment Pose2D::operator*(const LineSegment &lineSegment) const
{
	return LineSegment(*this * lineSegment.getStartPose(), lineSegment.getLength(), lineSegment.getExitAngle());
}

const LineSegment Pose2D::operator/(const LineSegment &lineSegment) const
{
	return LineSegment(*this / lineSegment.getStartPose(), lineSegment.getLength(), lineSegment.getExitAngle());
}

const Polygon::Ptr Pose2D::operator*(Polygon::ConstPtr polygon) const
{
	if (!polygon) {
		return Polygon::Ptr();
	}

	std::vector<Vector2d> transformedPoints;
	for (Vector2d p : polygon->getPoints()) {
		transformedPoints.push_back(*this * p);
	}

	return boost::make_shared<Polygon>(transformedPoints);
}

const Polygon::Ptr Pose2D::operator/(Polygon::ConstPtr polygon) const
{
	if (!polygon) {
		return Polygon::Ptr();
	}

	std::vector<Vector2d> transformedPoints;
	for (Vector2d p : polygon->getPoints()) {
		transformedPoints.push_back(*this / p);
	}

	return boost::make_shared<Polygon>(transformedPoints);
}

const Pose2D Pose2D::operator-(const Pose2D &other) const
{
	return Pose2D(*this) -= other;
}

const Pose2D Pose2D::operator/(const Pose2D &other) const
{
	return Pose2D(*this) /= other;
}

const Vector2d Pose2D::operator/(const Vector2d &vec) const
{
	return _angle / (vec - _position);
}

const BoundingBox2D Pose2D::operator*(const BoundingBox2D &box) const
{
	return BoundingBox2D(Pose2D(*this) * box.pose(), *this * box.minPoint(), *this * box.maxPoint());
}

const BoundingBox2D Pose2D::operator/(const BoundingBox2D &box) const
{
	return BoundingBox2D(Pose2D(*this) / box.pose(), *this / box.minPoint(), *this / box.maxPoint());
}

const AlignedBoundingBox3D Pose2D::operator*(const AlignedBoundingBox3D &box) const
{
	Vector2d origin2D(box.origin()(0), box.origin()(1));
	Vector2d minPoint2D(box.minPoint()(0), box.minPoint()(1));
	Vector2d maxPoint2D(box.maxPoint()(0), box.maxPoint()(1));
	origin2D = *this * origin2D;
	minPoint2D = _angle * minPoint2D;
	maxPoint2D = _angle * maxPoint2D;
	Vector3d origin3D = Vector3d(origin2D(0), origin2D(1), box.origin()(2));
	Vector3d minPoint3D = Vector3d(minPoint2D(0), minPoint2D(1), box.minPoint()(2));
	Vector3d maxPoint3D = Vector3d(maxPoint2D(0), maxPoint2D(1), box.maxPoint()(2));

	return AlignedBoundingBox3D(origin3D, minPoint3D, maxPoint3D);
}

const AlignedBoundingBox3D Pose2D::operator/(const AlignedBoundingBox3D &box) const
{
	Vector2d origin2D(box.origin()(0), box.origin()(1));
	Vector2d minPoint2D(box.minPoint()(0), box.minPoint()(1));
	Vector2d maxPoint2D(box.maxPoint()(0), box.maxPoint()(1));
	origin2D = *this / origin2D;
	minPoint2D = _angle / minPoint2D;
	maxPoint2D = _angle / maxPoint2D;
	Vector3d origin3D = Vector3d(origin2D(0), origin2D(1), box.origin()(2));
	Vector3d minPoint3D = Vector3d(minPoint2D(0), minPoint2D(1), box.minPoint()(2));
	Vector3d maxPoint3D = Vector3d(maxPoint2D(0), maxPoint2D(1), box.maxPoint()(2));

	return AlignedBoundingBox3D(origin3D, minPoint3D, maxPoint3D);
}

bool Pose2D::operator==(const Pose2D &other) const
{
	return _position == other._position && _angle == other._angle;
}

bool Pose2D::operator!=(const Pose2D &other) const
{
	return !(*this == other);
}

const Pose2D Pose2D::invert() const
{
	Vector2d inversePos(-1 * (_angle * _position));

	return Pose2D(inversePos, _angle.negate());
}

const double Pose2D::getDistanceTo(const Pose2D &other) const
{
	double deltax = _position(0) - other._position(0);
	double deltay = _position(1) - other._position(1);
	return std::sqrt(deltax * deltax + deltay * deltay);
}

const Angle Pose2D::getDeltaAngle(const Pose2D &other) const
{
	return other._angle - _angle;
}

const Vector2d Pose2D::getUnitVector() const
{
	return Vector2d(std::cos(_angle.rad()), std::sin(_angle.rad()));
}

const Angle Pose2D::getAngleTo(const Pose2D &other) const
{
	return getAngleTo(other.getPosition());
}

const Angle Pose2D::getAngleTo(const Vector2d &position) const
{
	Vector2d relative = *this / position;
	return Angle::to(relative);
}

// ========== STATIC HELPER METHODS ===========
const Pose2D Pose2D::average(const Pose2D &p1, const Pose2D &p2, const double &weight1, const double &weight2)
{
	double divider = weight1 + weight2;
	double x = (p1.x() * weight1 + p2.x() * weight2) / divider;
	double y = (p1.y() * weight1 + p2.y() * weight2) / divider;

	return Pose2D(x, y, Angle::average(p1.getAngle(), p2.getAngle(), weight1, weight2));
}

const Pose2D Pose2D::average(const std::vector<Pose2D> &poses)
{
	size_t size = poses.size();
	if (size == 0) {
		return Pose2D();
	} else if (size == 1) {
		return poses[0];
	}

	double xSum = 0;
	double ySum = 0;
	double angleSum = 0;

	for (Pose2D p : poses) {
		xSum += p.x();
		ySum += p.y();
		angleSum += p.getAngle().rad() + M_PI;
	}

	return Pose2D(xSum / size, ySum / size, Angle::rad((angleSum / size) - M_PI));
}

const Pose2D Pose2D::interpolate(const Pose2D &startPose, const Pose2D &endPose, const double &progress)
{
	if (progress <= 0) {
		return startPose;
	} else if (progress >= 1) {
		return endPose;
	}

	Vector2d diff = endPose.getPosition() - startPose.getPosition();
	Angle angle = Angle::interpolate(startPose.getAngle(), endPose.getAngle(), progress);

	return Pose2D(startPose.getPosition() + (diff * progress), angle);
}

namespace taco
{
std::ostream &operator<<(std::ostream &os, const Pose2D &pose)
{
	os << pose.x() << " " << pose.y() << " " << pose.getAngle();
	return os;
}
}
