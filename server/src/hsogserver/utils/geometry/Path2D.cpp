#include "Path2D.h"
#include "Geometry.h"

using namespace taco;
using namespace Eigen;

Path2D::Path2D() : _length(0), _startPose(0, 0), _endPose(0, 0)
{
}

Path2D::~Path2D()
{
}

const double &Path2D::getLength() const
{
	return _length;
}

const Pose2D &Path2D::getStartPose() const
{
	return _startPose;
}

const Pose2D &Path2D::getEndPose() const
{
	return _endPose;
}

const std::vector<Pose2D> &Path2D::getWayPoints() const
{
	return _wayPoints;
}

const Pose2D Path2D::getLastWayPoint() const
{
	if (_wayPoints.size() == 0) {
		return Pose2D();
	}

	return _wayPoints.back();
}

const std::vector<LineSegment> &Path2D::getLineSegments() const
{
	return _segments;
}

void Path2D::addWayPoint(const Pose2D &wayPoint)
{
	if (_wayPoints.size() == 0) {
		// First way point
		_wayPoints.push_back(wayPoint);
		_startPose = wayPoint;
		_endPose = wayPoint;
		return;
	}

	Pose2D previousWayPoint = _wayPoints.back();
	_wayPoints.push_back(wayPoint);
	_length += Geometry::constructDoubleArc(previousWayPoint, wayPoint, _segments);
	_endPose = wayPoint;
}

const double Path2D::getPositionOnPath(const Vector2d &position) const
{
	if (_wayPoints.size() < 2) {
		return 0;
	}

	double extension = 0;
	bool foundSegment = false;

	for (LineSegment seg : _segments) {
		if (seg.isOnSegment(position)) {
			extension += seg.getExtension(position);
			foundSegment = true;
			break;
		} else {
			extension += seg.getLength();
		}
	}

	if (!foundSegment) {
		if (_startPose.getDistanceTo(position) < _endPose.getDistanceTo(position)) {
			extension = 0;
		} else {
			extension = _length;
		}
	}

	return extension;
}

const Pose2D Path2D::interpolateWayPose(const double &length) const
{
	if (length <= 0 || _wayPoints.size() < 2) {
		return _startPose;
	} else if (length >= _length) {
		return _endPose;
	}

	size_t idx = 0;
	double remainingLength = length;
	while (idx < _segments.size() - 1 && remainingLength > _segments[idx].getLength()) {
		remainingLength -= _segments[idx].getLength();
		idx++;
	}

	return _segments[idx].interpolatePose(remainingLength);
}
