#include "LineSegment.h"
#include "Circle2D.h"
#include "Geometry.h"

#include <cmath>
#include <limits>

#define TWO_PI 2 * M_PI

using namespace taco;
using namespace Eigen;

LineSegment::LineSegment(const Pose2D &startPose, const double &length, const double &exitAngle)
	: _startPose(startPose), _endPose(), _length(length), _exitAngle(exitAngle)
{
	if (_length < 0) {
		_length *= -1;
	}

	if (std::abs(_exitAngle) > TWO_PI) {
		_exitAngle = std::fmod(_exitAngle, TWO_PI);
	}

	_endPose = _startPose * Geometry::calculateArcPose(_length, _exitAngle);
}

LineSegment::LineSegment(
		const Circle2D &circle, const Vector2d &startPos, const Vector2d &endPos, const bool &clockwise)
	: LineSegment(circle, Angle::to(startPos - circle.origin()), Angle::to(endPos - circle.origin()), clockwise)
{
}

LineSegment::LineSegment(const Circle2D &circle, const Angle &startAngle, const Angle &endAngle, const bool &clockwise)
{
	Angle diff = endAngle - startAngle;

	if (clockwise && diff.rad() > 0) {
		_exitAngle = diff.rad() - TWO_PI;
	} else if (!clockwise && diff.rad() < 0) {
		_exitAngle = diff.rad() + TWO_PI;
	} else {
		_exitAngle = diff.rad();
	}

	_length = std::abs(_exitAngle) * circle.radius();
	_startPose =
			Pose2D(circle.getPointOnCircle(startAngle), startAngle + (clockwise ? Angle::Deg_N90() : Angle::Deg_90()));
	_endPose = _startPose * Geometry::calculateArcPose(_length, _exitAngle);
}

LineSegment::~LineSegment()
{
}

const Pose2D &LineSegment::getStartPose() const
{
	return _startPose;
}

const Pose2D &LineSegment::getEndPose() const
{
	return _endPose;
}

const double &LineSegment::getLength() const
{
	return _length;
}

const double &LineSegment::getExitAngle() const
{
	return _exitAngle;
}

const double LineSegment::getRadius() const
{
	if (_exitAngle == 0) {
		return std::numeric_limits<double>::max();
	}

	return _length / _exitAngle;
}

const Pose2D LineSegment::interpolatePose(const double &length) const
{
	double factor = length / _length;
	return _startPose * Geometry::calculateArcPose(length, factor * _exitAngle);
}

void LineSegment::interpolateIntermediatePoses(const size_t &howMany, std::vector<Pose2D> &result) const
{
	Geometry::interpolateArcPoses(_length, _exitAngle, howMany, result, _startPose);
}

const bool LineSegment::isOnSegment(const Vector2d &position) const
{
	Vector2d toStart = _startPose / position;
	Vector2d toEnd = _endPose / position;

	if (std::fabs(_exitAngle) <= M_PI) {
		return toStart(0) > 0 && toEnd(0) < 0.001;
	} else {
		return toStart(0) > 0 || toEnd(0) < 0.001;
	}
}

const double LineSegment::getExtension(const Vector2d &position) const
{
	if (std::fabs(_exitAngle) < 0.000001) {
		// Straight segment
		Vector2d localPos = _startPose / position;
		return localPos(0);
	} else {
		double radius = _length / _exitAngle;
		bool leftCurve = radius > 0;
		Pose2D circlePose =
				_startPose * Pose2D(0, radius, leftCurve ? Angle::deg(-90.0000001) : Angle::deg(90.0000001));
		double angle = circlePose.getAngleTo(position).rad();
		if (leftCurve && angle < 0) {
			angle += TWO_PI;
		} else if (!leftCurve && angle > 0) {
			angle -= TWO_PI;
		}

		return _length * angle / _exitAngle;
	}
}

const double LineSegment::getDistanceTo(const Vector2d &position) const
{
	double extension = getExtension(position);
	Pose2D closestPose = interpolatePose(extension);
	return closestPose.getDistanceTo(position);
}

namespace taco
{
std::ostream &operator<<(std::ostream &os, const taco::LineSegment &segment)
{
	os << "l: " << segment.getLength() << " e-a: " << segment.getExitAngle() << " from " << segment.getStartPose()
	   << " to " << segment.getEndPose();
	return os;
}
}
