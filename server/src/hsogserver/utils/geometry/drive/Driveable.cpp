#include "Driveable.h"

using namespace taco;
using namespace std;
using namespace Eigen;

Driveable Driveable::INVALID(nullptr);

Driveable::Driveable()
{
	//   _valid = false;
}

Driveable::Driveable(IManeuverGeometry::Ptr geometry)
{
	if (geometry) {
		_geometries.push_back(geometry);
		_valid = true;
	}
}

Driveable::Driveable(IManeuverGeometry::Ptr geometry1, IManeuverGeometry::Ptr geometry2, const Pose2D &intercept)
{
	_geometries.push_back(geometry1);
	_geometries.push_back(geometry2);
	_intercept = intercept;
	_valid = true;
}

Driveable::~Driveable()
{
}

Pose2D Driveable::getClosestPose(const Vector2d &point)
{
	IManeuverGeometry::Ptr geometry;

	if (!isValid())
		return Pose2D();

	if (_geometries.size() == 1) {
		geometry = _geometries.at(0);
		Pose2D pose = geometry->getClosestPose(point);
		return pose;
	}

	Vector2d pointTransformed = _intercept / point;

	if (pointTransformed(0) < 0) {
		geometry = _geometries.at(0);
	} else {
		geometry = _geometries.at(1);
	}

	return geometry->getClosestPose(point);
}

vector<Vector2d> Driveable::getTrail(const Vector2d &start, const Vector2d &end)
{
	IManeuverGeometry::Ptr geometry;

	if (!isValid())
		return vector<Vector2d>();

	if (_geometries.size() == 1) {
		geometry = _geometries.at(0);
		return geometry->getTrail(start, end);
	}
	geometry = _geometries.at(0);
	vector<Vector2d> points = geometry->getTrail(start, _intercept.getPosition());
	geometry = _geometries.at(1);
	for (Vector2d point : geometry->getTrail(_intercept.getPosition(), end)) {
		points.push_back(point);
	}
	//   vector<Vector2d> points2 = geometry->getTrail(_intercept.getPosition(), end);
	//   points.insert(points2);
	return points;
}

const Driveable &Driveable::Invalid()
{
	return INVALID;
}

bool Driveable::isValid()
{
	if (_geometries.size() == 0)
		return false;
	else
		return _valid;
}
