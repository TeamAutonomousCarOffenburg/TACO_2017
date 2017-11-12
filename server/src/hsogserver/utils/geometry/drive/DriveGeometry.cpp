#include "DriveGeometry.h"
#include "utils/geometry/Geometry.h"

#include <cmath>
#include <limits>

using namespace taco;
using namespace Eigen;

#define THRESHOLD 5
#define MINIMUM_RADIUS 0.5

const Pose2D DriveGeometry::calculateCurvePose(
		const double &leftWheelDistance, const double &rightWheelDistance, const double &wheelSpacing)
{
	double x, y, alpha;

	if (leftWheelDistance == rightWheelDistance) {
		// Both wheels spin in the same direction with the same speed, thus we are driving in a straight line.
		x = 0;
		y = leftWheelDistance;
		alpha = 0;
	} else if (leftWheelDistance == -rightWheelDistance) {
		// Both wheels spin at the same speed, but in different directions, thus we are turing on spot.
		x = 0;
		y = 0;
		alpha = 2 * leftWheelDistance / wheelSpacing;
	} else {
		double innerWheelDistance = leftWheelDistance;
		double outerWheelDistance = rightWheelDistance;
		int sideFactor = 1;

		if (std::abs(leftWheelDistance) > std::abs(rightWheelDistance)) {
			innerWheelDistance = rightWheelDistance;
			outerWheelDistance = leftWheelDistance;
			sideFactor = -1;
		}

		if (leftWheelDistance * rightWheelDistance >= 0) {
			// Both wheels spin in the same direction, thus the origin of the driving circle is outside (left or right)
			// the car.

			double diffDistance = outerWheelDistance - innerWheelDistance;
			double r = (wheelSpacing * innerWheelDistance / diffDistance) + (wheelSpacing / 2);

			alpha = sideFactor * (diffDistance / wheelSpacing);
			x = sideFactor * (std::cos(alpha) * r - r);
			y = sideFactor * (std::sin(alpha) * r);
		} else {
			// The wheels spin in different directions, thus the origin of the driving circle is between the two wheels.

			double distanceSum = innerWheelDistance + outerWheelDistance;
			double r = (wheelSpacing / 2) - (wheelSpacing * innerWheelDistance / distanceSum);

			alpha = sideFactor * (distanceSum / wheelSpacing);
			x = sideFactor * (std::cos(alpha) * r - r);
			y = sideFactor * (std::sin(alpha) * r);
		}
	}

	return Pose2D(x, y, Angle::rad(alpha));
}

const double DriveGeometry::calculateCurveRadius(const Angle &steeringAngle, const double &axleSpacing)
{
	if (std::abs(steeringAngle.rad()) >= M_PI / 2) {
		return 0;
	} else if (steeringAngle.rad() == 0) {
		return std::numeric_limits<double>::max();
	}

	return axleSpacing / std::tan(steeringAngle.rad());
}

const Angle DriveGeometry::calculateSteeringAngle(const double &curveRadius, const double &axleSpacing)
{
	if (curveRadius == 0) {
		return Angle::Deg_90();
	} else if (curveRadius == -0) {
		return Angle::Deg_N90();
	}

	return Angle::rad(std::atan(std::abs(axleSpacing) / curveRadius));
}

const Driveable DriveGeometry::getLineBeforeCircleGeometry(const Pose2D &carPose, const Pose2D &nextWayPoint)
{
	Pose2D nextWayPointTransformed = carPose / nextWayPoint;
	if ((nextWayPointTransformed.y() > 0 && nextWayPointTransformed.getAngle().rad() < 0) ||
			(nextWayPointTransformed.y() < 0 && nextWayPointTransformed.getAngle().rad() > 0))
		return Driveable::Invalid();

	double x = Geometry::getTouchingPoint(nextWayPointTransformed);
	// if first point on circle is behind car pose -> circle starts behind car pose
	if (x < 0)
		return Driveable::Invalid();

	Vector2d lineEnd = carPose * Vector2d(x, 0);
	Pose2D intercept(lineEnd, carPose.getAngle());
	Circle2D circle(nextWayPoint, lineEnd);
	// if circle origin is behind nextWayPoint, path is in wrong direction
	if ((nextWayPoint / circle.origin())(0) < 0)
		return Driveable::Invalid();
	// if intercept is behind car pose
	//   else if((carPose / intercept).x() > 0)
	//     return Driveable::Invalid();
	// if angle is same
	else if (fabs(carPose.getAngle().rad() - nextWayPoint.getAngle().rad()) < 0.0001)
		return Driveable::Invalid();
	// if line points in wrong direction: )_ instead of _)
	else if (circle.getDirectionOfRotation(nextWayPoint) != circle.getDirectionOfRotation(intercept))
		return Driveable::Invalid();
	// if radius is too small
	else if (circle.radius() < MINIMUM_RADIUS)
		return Driveable::Invalid();
	else
		return Driveable(IManeuverGeometry::Ptr(new Line2D(carPose.getPosition(), lineEnd)),
				IManeuverGeometry::Ptr(new Circle2D(circle)), intercept);
}

const Driveable DriveGeometry::getCircleBeforeLineGeometry(const Pose2D &carPose, const Pose2D &nextWayPoint)
{
	Pose2D carPoseTransformed = nextWayPoint / carPose;
	if ((carPoseTransformed.y() > 0 && carPoseTransformed.getAngle().rad() < 0) ||
			(carPoseTransformed.y() < 0 && carPoseTransformed.getAngle().rad() > 0))
		return Driveable::Invalid();

	double x = Geometry::getTouchingPoint(carPoseTransformed);
	// if point is in front of next way point -> circle ends behind nextWayPoint
	if (x > 0)
		return Driveable::Invalid();

	Vector2d lineStart = nextWayPoint * Vector2d(x, 0);
	Circle2D circle(carPose, lineStart);
	Pose2D intercept(lineStart, nextWayPoint.getAngle());
	// if circle origin is behind carPose, path is in wrong direction
	if ((carPose / circle.origin())(0) < 0)
		return Driveable::Invalid();
	// if angle is same
	else if (fabs(carPose.getAngle().rad() - nextWayPoint.getAngle().rad()) < 0.0001)
		return Driveable::Invalid();
	// if intercept is behind next way point
	//   else if((nextWayPoint / intercept).x() < 0)
	//     return Driveable::Invalid();
	// if line points in wrong direction: )_ instead of _)
	else if (circle.getDirectionOfRotation(intercept) != circle.getDirectionOfRotation(carPose))
		return Driveable::Invalid();
	// if radius is too small
	else if (circle.radius() < MINIMUM_RADIUS)
		return Driveable::Invalid();
	else
		return Driveable(IManeuverGeometry::Ptr(new Circle2D(circle)),
				IManeuverGeometry::Ptr(new Line2D(lineStart, nextWayPoint.getPosition())), intercept);
}

const Driveable DriveGeometry::getCircle(const Pose2D &carPose, const Pose2D &nextWayPoint)
{
	Circle2D circle(nextWayPoint, carPose.getPosition());

	if (fabs((circle.slope(carPose.getPosition()) - carPose.getAngle()).deg()) > THRESHOLD &&
			fabs((circle.slope(carPose.getPosition()).opposite() - carPose.getAngle()).deg()) > THRESHOLD)
		return Driveable::Invalid();
	else if (circle.getDirectionOfRotation(nextWayPoint) != circle.getDirectionOfRotation(carPose))
		return Driveable::Invalid();
	else if (circle.radius() < MINIMUM_RADIUS)
		return Driveable::Invalid();
	else
		return Driveable(IManeuverGeometry::Ptr(new Circle2D(circle)));
}

const Driveable DriveGeometry::DriveGeometry::getLine(const Pose2D &carPose, const Pose2D &nextWayPoint)
{
	Line2D line(carPose.getPosition(), nextWayPoint.getPosition());
	// if differences of angle from carPose and nextWayPoint is higher than threshold
	if (fabs((line.getAngle() - carPose.getAngle()).deg()) > THRESHOLD ||
			fabs((line.getAngle() - nextWayPoint.getAngle()).deg()) > THRESHOLD)
		return Driveable::Invalid();
	else
		return Driveable(IManeuverGeometry::Ptr(new Line2D(line)));
}

const Driveable DriveGeometry::getSCurveBeforeLineGeometry(const Pose2D &carPose, const Pose2D &nextWayPoint)
{
	double angleDiff = fabs((carPose.getAngle() - nextWayPoint.getAngle()).deg());
	// catch threshold
	if (angleDiff > 45 && angleDiff < 135)
		return Driveable::Invalid();

	Cosine geometry = getConnectingCosine(nextWayPoint, carPose);
	geometry.normalizeRatioToPoint(carPose.getPosition());
	Pose2D nextWayPointTransformed = geometry.origin() / nextWayPoint;
	// check if curve is too long for next way point
	if (nextWayPointTransformed.x() < 0)
		return Driveable::Invalid();
	// high elongation ^= high steering angle
	else if (geometry.elongation() > 14.0)
		return Driveable::Invalid();

	// line starts right or left of origin of cosine
	Vector2d lineStart = geometry.origin() * Vector2d(0, geometry.spreading());
	Line2D line(lineStart, nextWayPoint.getPosition());

	return Driveable(IManeuverGeometry::Ptr(new Cosine(geometry)), IManeuverGeometry::Ptr(new Line2D(line)),
			Pose2D(lineStart, nextWayPoint.getAngle()));
}

const Cosine DriveGeometry::getConnectingCosine(const Pose2D &directionLocal, const Pose2D &other)
{
	Pose2D otherTransformed = directionLocal / other;

	double slope = tan(otherTransformed.getAngle().rad());
	bool directionChanged = false;
	if (otherTransformed.y() > 0) {
		directionChanged = true;
		slope *= -1;
	}
	double x = asin(slope) + M_PI;
	double yDiff = fabs(otherTransformed.y());
	double yOffset = fabs(cos(x)) + 1;
	double spreading = yDiff / yOffset;
	if (directionChanged)
		spreading *= -1;

	double periodLength = fabs((2.0 * M_PI * otherTransformed.x()) / x);
	double elongation = (2 * M_PI) / periodLength;

	Pose2D originTransformed = Pose2D(0, -spreading, Angle());
	Pose2D origin = directionLocal * originTransformed;
	return Cosine(origin, elongation, spreading);
}
