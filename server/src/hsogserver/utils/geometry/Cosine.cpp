#include "Cosine.h"

#include "Geometry.h"

using namespace taco;
using namespace std;
using namespace Eigen;

Cosine::Cosine(const Pose2D &origin, const double &elongation, const double &spreading)
	: _origin(origin), _elongation(elongation), _spreading(spreading)
{
	validate();
}

Cosine::Cosine() : _origin(Pose2D()), _elongation(1), _spreading(1)
{
	validate();
}

Cosine::~Cosine()
{
}

Pose2D Cosine::getClosestPose(const Vector2d &point)
{
	Vector2d pointTransformed = _origin / point;
	double y;
	if (pointTransformed(0) > 0) {
		y = _spreading;
	} else {
		y = _spreading * cos(_elongation * pointTransformed(0));
	}
	double slope = _spreading * (-sin(pointTransformed(0) * _elongation)) * _elongation;
	double angle = atan(slope);
	return _origin * Pose2D(pointTransformed(0), y, Angle(angle));
}

double &Cosine::elongation()
{
	return _elongation;
}

Pose2D &Cosine::origin()
{
	return _origin;
}

double &Cosine::spreading()
{
	return _spreading;
}

vector<Vector2d> Cosine::getTrail(const Vector2d &start, const Vector2d &end)
{
	double xDiff = end(0) - start(0);
	Vector2d deltaVector = Vector2d(xDiff * 0.1, 0);
	std::vector<Vector2d> points;
	Vector2d point = getClosestPose(start).getPosition();
	int i = 0;
	do {
		point = point + deltaVector;
		point = getClosestPose(point).getPosition();
		points.push_back(point);
	} while (++i < 10);
	return points;
}

void Cosine::validate()
{
	if (_spreading == 0)
		_spreading = 1.0;

	if (_elongation == 0)
		_elongation = 1.0;
}

void Cosine::normalizeRatioToPoint(Vector2d point)
{
	Vector2d pointTransformed = _origin / point;

	double periodLength = (2 * M_PI) / _elongation;
	double newPeriodLength = (2 * M_PI) / fabs(1 / _spreading);

	double newX = (pointTransformed(0) * newPeriodLength) / periodLength;
	double offsetX = pointTransformed(0) - newX;
	Vector2d offset;
	if (fabs(offsetX) < 0.00001) {
		return;
	} else if (offsetX > 0)
		offset = _origin.getAngle().getVector(offsetX);
	else
		offset = _origin.getAngle().opposite().getVector(fabs(offsetX));

	_origin.setPosition(_origin.getPosition() + offset);
	_elongation = 1 / fabs(_spreading);
}
