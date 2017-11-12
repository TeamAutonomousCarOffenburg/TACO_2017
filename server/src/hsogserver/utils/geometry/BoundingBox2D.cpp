#include "BoundingBox2D.h"

using namespace taco;

BoundingBox2D::BoundingBox2D(const Pose2D &pose, const Eigen::Vector2d &minPoint, const Eigen::Vector2d &maxPoint)
	: _pose(pose), _minPoint(minPoint), _maxPoint(maxPoint)
{
}
BoundingBox2D::~BoundingBox2D()
{
}

const Pose2D BoundingBox2D::pose() const
{
	return _pose;
}

const Eigen::Vector2d BoundingBox2D::maxPoint() const
{
	return _maxPoint;
}

const Eigen::Vector2d BoundingBox2D::minPoint() const
{
	return _minPoint;
}