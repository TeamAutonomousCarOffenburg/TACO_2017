#include "DriveCalculator.h"
#include "DriveGeometry.h"

#include <cmath>

using namespace taco;

DriveCalculator::DriveCalculator() : _wheelSpacing(0.26), _axleSpacing(0.36)
{
}

DriveCalculator::DriveCalculator(const double &wheelSpacing, const double &axleSpacing)
	: _wheelSpacing(wheelSpacing), _axleSpacing(axleSpacing)
{
}

DriveCalculator::~DriveCalculator()
{
}

const Pose2D DriveCalculator::calculateCurvePose(
		const double &leftWheelDistance, const double &rightWheelDistance) const
{
	return DriveGeometry::calculateCurvePose(leftWheelDistance, rightWheelDistance, _wheelSpacing);
}

const double DriveCalculator::calculateCurveRadius(const Angle &steeringAngle) const
{
	return DriveGeometry::calculateCurveRadius(steeringAngle, _axleSpacing);
}

const Angle DriveCalculator::calculateSteeringAngle(const double &curveRadius) const
{
	return DriveGeometry::calculateSteeringAngle(curveRadius, _axleSpacing);
}
