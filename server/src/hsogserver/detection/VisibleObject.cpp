#include "VisibleObject.h"

using namespace taco;
using namespace Eigen;

VisibleObject::VisibleObject(const std::string &name, const Pose2D &pose, Polygon::Ptr bounds)
	: _name(name), _pose(pose), _bounds(bounds)
{
	if (!_bounds) {
		_bounds = boost::make_shared<Polygon>(
				Vector2d(0.05, 0.05), Vector2d(0.05, -0.05), Vector2d(-0.05, -0.05), Vector2d(-0.05, 0.05));
	}
}

VisibleObject::~VisibleObject()
{
}

void VisibleObject::update(const Pose2D &pose)
{
	_pose = pose;
}

const std::string &VisibleObject::getName() const
{
	return _name;
}

const Pose2D &VisibleObject::getPose() const
{
	return _pose;
}

void VisibleObject::setPose(const Pose2D &pose)
{
	_pose = pose;
}

Polygon::ConstPtr VisibleObject::getBoundingPoly() const
{
	return boost::dynamic_pointer_cast<const Polygon>(_bounds);
}

void VisibleObject::setBoundingPoly(Polygon::Ptr bounds)
{
	if (bounds) {
		_bounds = bounds;
	}
}
