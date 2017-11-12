#include "Geometry.h"

#include "Angle.h"
#include "Circle2D.h"
#include "Line2D.h"
#include "LineSegment.h"
#include "Polygon.h"
#include "Pose2D.h"
#include "Pose3D.h"

using namespace taco;
using namespace Eigen;

const Angle Geometry::getDelta(const Vector3d &vector)
{
	double length = getNorm(vector);
	if (length == 0) {
		return Angle::Zero();
	}

	return Angle::rad(std::asin(vector(2) / length));
}

Angle Geometry::getHorizontalAngle(const Quaterniond &orientation)
{
	Vector3d unitX = orientation * Vector3d::UnitX();
	return Angle::to(unitX(0), unitX(1));
}

const double Geometry::getDistance(const double &p1x, const double &p1y, const double &p2x, const double &p2y)
{
	double xDiff = p2x - p1x;
	double yDiff = p2y - p1y;
	return std::sqrt(xDiff * xDiff + yDiff * yDiff);
}

const bool Geometry::checkParallel(const Line2D &line1, const Line2D &line2)
{
	double xDiff1 = line1.getExtensionVector()(0);
	double xDiff2 = line2.getExtensionVector()(0);

	if (xDiff1 == 0 && xDiff2 == 0) {
		// Both lines are parallel to y axis
		return true;
	} else if (xDiff1 == 0 || xDiff2 == 0) {
		// They are not both parallel to y axis, thus if one is, they are not parallel
		return false;
	} else {
		// Neither line is parallel to y axis, thus compare the slope of both lines
		return line1.getExtensionVector()(1) / xDiff1 == line2.getExtensionVector()(1) / xDiff2;
	}
}

const bool Geometry::intersectionPoint(const Line2D &line1, const Line2D &line2, Vector2d &result)
{
	// Check if lines are parallel
	if (checkParallel(line1, line2)) {
		return false;
	}

	if (line1.getExtensionVector()(0) == 0) {
		// line1 is y parallel, thus the intersection is at x-value: line1.getStart()(0)
		double x = line1.getStart()(0);
		result(0) = x;
		result(1) = line2.yValue(x);
	} else if (line2.getExtensionVector()(0) == 0) {
		// line2 is y parallel, thus the intersection is at x-value: line2.getStart()(0)
		double x = line2.getStart()(0);
		result(0) = x;
		result(1) = line1.yValue(x);
	} else {
		double m1 = line1.m();
		double m2 = line2.m();
		double b1 = line1.getStart()(1) - m1 * line1.getStart()(0);
		double b2 = line2.getStart()(1) - m2 * line2.getStart()(0);

		double x = (b2 - b1) / (m1 - m2);
		result(0) = x;
		result(1) = m1 * x + b1;
	}

	return true;
}

void Geometry::intersectionPoints(const Line2D &line, const Circle2D &circle, std::vector<Vector2d> &result)
{
	Pose2D linePose(line.getStart(), Angle::to(line.getExtensionVector()));

	Vector2d localOrigin = linePose / circle.origin();

	// Find intersection points with x-axis
	double xAxisDistance = std::abs(localOrigin(1)) - circle.radius();
	if (xAxisDistance == 0) {
		// There is only one intersection point (the line is a tangent of the circle)
		result.push_back(linePose * Vector2d(localOrigin(0), 0));
	} else if (xAxisDistance < 0) {
		// There are be two intersection points
		double remainder = std::sqrt((circle.radius() * circle.radius()) - (localOrigin(1) * localOrigin(1)));
		result.push_back(linePose * Vector2d(localOrigin(0) + remainder, 0));
		result.push_back(linePose * Vector2d(localOrigin(0) - remainder, 0));
	} else {
		// There are no intersection points
	}
}

void Geometry::intersectionPoints(const Circle2D &circle1, const Circle2D &circle2, std::vector<Vector2d> &result)
{
	if (circle1.origin() == circle2.origin()) {
		// No intersection points with same origin
		return;
	}

	Vector2d centerVec = circle2.origin() - circle1.origin();
	Pose2D circle1Pose(circle1.origin(), Angle::to(centerVec));
	Vector2d c2Local = circle1Pose / circle2.origin();
	double r1 = circle1.radius();
	double r2 = circle2.radius();

	if ((r1 + r2 == c2Local(0)) || (c2Local(0) + r2 == r1)) {
		// There exists only one intersection point to the right of circle 1
		result.push_back(circle1Pose * Vector2d(r1, 0));
	} else if (c2Local(0) + r1 == r2) {
		// There exists only one intersection point to the left of circle 1
		result.push_back(circle1Pose * Vector2d(-r1, 0));
	} else if (c2Local(0) > r1 + r2 || r1 > c2Local(0) + r2 || r2 > c2Local(0) + r1) {
		// No intersection points
	} else {
		// There are two intersection points
		double x = ((r1 * r1) - (r2 * r2) + (c2Local(0) * c2Local(0))) / (2 * c2Local(0));
		double y = std::sqrt((r1 * r1) - (x * x));
		result.push_back(circle1Pose * Vector2d(x, y));
		result.push_back(circle1Pose * Vector2d(x, -y));
	}
}

void Geometry::commonTangents(const Circle2D &circle1, const Circle2D &circle2, std::vector<Line2D> &result)
{
}

void Geometry::slope(const Circle2D &circle, const Vector2d &point, Angle &result)
{
	if (!circle.isOnCircle(point)) {
		throw "Point is not on circle";
	}

	// catch limits
	if (circle.radius() + circle.origin()(0) == point(0) || circle.radius() + circle.origin()(0) == point(0)) {
		result = Angle(0.5 * M_PI);
		return;
	}

	// calculate slope on upper semicircle
	double slope = -0.5 * sqrt(1 / (pow(circle.radius(), 2) - pow(point(0) - circle.origin()(0), 2))) *
				   (2 * point(0) - 2 * circle.origin()(0));

	// check if point is on lower semicircle
	if (circle.origin()(1) > point(1)) {
		slope *= -1;
	}
	result = Angle(atan(slope));
}

double Geometry::getTouchingPoint(const Pose2D &circleSideTransformed)
{
	Vector2d targetNormal = circleSideTransformed.getAngle().normal().getVector();
	// check if normal has correct direction: if normal is wrong t = -t
	// get intersept with x axis
	//     Vector2d direction = circleSideTransformed.getAngle().getVector();
	//     double tmp = - circleSideTransformed.y() / direction(1);
	//     double xIntersept = circleSideTransformed.x() + tmp * direction(0);
	//
	//     if((xIntersept < 0 && targetNormal(0) < 0) || (xIntersept > 0 && targetNormal(0) > 0))
	//     {
	//       targetNormal  *= -1;
	//     }

	double yNormal = 1;
	if (circleSideTransformed.y() < 0) {
		targetNormal *= -1;
		yNormal = -1;
	}

	double t = circleSideTransformed.y() / (yNormal - targetNormal(1));
	return circleSideTransformed.x() + t * targetNormal(0);
}

Pose2D Geometry::calculateArcPose(const double &length, const Angle &exitAngle)
{
	return calculateArcPose(length, exitAngle.rad());
}

Pose2D Geometry::calculateArcPose(const double &length, const double &exitAngle)
{
	double x = length;
	double y = 0;

	if (exitAngle != 0) {
		double radius = length / exitAngle;
		x = sin(exitAngle) * radius;
		y = radius * (1 - cos(exitAngle));
	}

	return Pose2D(x, y, Angle::rad(exitAngle));
}

void Geometry::interpolateArcPoses(const double &length, const double &exitAngle, const size_t &howMany,
		std::vector<Pose2D> &result, const Pose2D &startPose)
{
	if (exitAngle == 0) {
		double step = length / (howMany + 1);

		for (size_t i = 1; i <= howMany; i++) {
			result.push_back(startPose * Pose2D(step * i, 0));
		}
	} else {
		double radius = length / exitAngle;
		double step = exitAngle / (howMany + 1);
		double runningAngle;
		Pose2D pose;

		for (size_t i = 1; i <= howMany; i++) {
			runningAngle = step * i;
			pose = Pose2D(sin(runningAngle) * radius, radius * (1 - cos(runningAngle)), Angle::rad(runningAngle));
			result.push_back(startPose * pose);
		}
	}
}

double Geometry::constructDoubleArc(const Pose2D &start, const Pose2D &end, std::vector<LineSegment> &result)
{
	Pose2D localEnd = start / end;
	Vector2d r = (localEnd.getAngle() - Angle::Deg_90()).getVector();

	double squarePart = (r(0) * r(0)) + (r(1) * r(1)) - 2 * r(1) - 3;
	double linearPart = 2 * (localEnd.x() * r(0) + localEnd.y() * r(1) - localEnd.y());
	double offset = localEnd.x() * localEnd.x() + localEnd.y() * localEnd.y();

	if (std::fabs(squarePart) < 0.000001) {
		if (std::fabs(linearPart) < 0.000001) {
			// No mathematical or numerically stable solution when poses are colinear,
			// thus connect poses with a straight line segment
			result.push_back(LineSegment(start, localEnd.x(), 0));
			return localEnd.x();
		} else {
			// Single solution
			double radius = -1 * offset / linearPart;
			return constructDoubleArc(start, end, radius, result);
		}
	} else {
		// Double solution
		double pHalf = 0.5 * linearPart / squarePart;
		double q = offset / squarePart;
		double remainder = std::sqrt(pHalf * pHalf - q);

		double radius1 = -1 * pHalf + remainder;
		double radius2 = -1 * pHalf - remainder;

		std::vector<LineSegment> res1;
		double length1 = constructDoubleArc(start, end, radius1, res1);

		std::vector<LineSegment> res2;
		double length2 = constructDoubleArc(start, end, radius2, res2);

		if (length1 < length2) {
			for (LineSegment seg : res1) {
				result.push_back(seg);
			}
			return length1;
		} else {
			for (LineSegment seg : res2) {
				result.push_back(seg);
			}
			return length2;
		}
	}
}

const double Geometry::getDistanceToLine(const Line2D &line, const Vector2d &point)
{
	return getDistanceToLine(line.getStart(), line.getEnd(), point);
}

const double Geometry::getDistanceToLine(const Vector2d &lineStart, const Vector2d &lineEnd, const Vector2d &point)
{
	Vector2d v = lineEnd - lineStart;
	Vector2d w = point - lineStart;

	double c1 = w.dot(v);

	// c1 <= 0 --> before line-start
	if (c1 > 0) {
		double c2 = v.dot(v);

		if (c2 <= c1) {
			// after line-end
			w = point - lineEnd;
		} else {
			w = point - (lineStart + (v * (c1 / c2)));
		}
	}

	return getNorm(w);
}

const bool Geometry::isInsidePolygon(const std::vector<Vector2d> &polygon, const Vector2d &point)
{
	size_t size = polygon.size();
	if (size <= 2) {
		return false;
	}

	size_t idx0 = size - 1;
	size_t idx1 = 0;
	double x2 = point(0);
	double y2 = point(1);
	for (idx1 = 0; idx1 < size;) {
		double x0 = polygon[idx0](0);
		double y0 = polygon[idx0](1);
		double x1 = polygon[idx1](0);
		double y1 = polygon[idx1](1);

		// if points are clockwise then the determinant has to be positive
		// ----1 | x0 y0 1 |
		// A = - | x1 y1 1 |
		// ----2 | x2 y2 1 |
		double result = 0.5 * (x1 * y2 - y1 * x2 - x0 * y2 + y0 * x2 + x0 * y1 - y0 * x1);
		if (result > 0) {
			return false;
		}

		idx0 = idx1;
		idx1++;
	}

	return true;
}

const double Geometry::getDistance(const std::vector<Vector2d> &polygon, const Vector2d &point)
{
	size_t size = polygon.size();
	if (size == 0) {
		return 0;
	}

	double minDistance = getDistance(polygon[0], point);
	double distance;
	for (size_t idx0 = size - 1, idx1 = 0; idx1 < size; idx0 = idx1++) {
		distance = getDistanceToLine(polygon[idx0], polygon[idx1], point);
		if (distance < minDistance) {
			minDistance = distance;
		}
	}

	return minDistance;
}

const Line2D Geometry::getClosestPolyLine(const std::vector<Vector2d> &polygon, const Vector2d &point)
{
	size_t size = polygon.size();
	if (size <= 1) {
		return Line2D();
	}

	double minDistance = getDistance(polygon[0], point);
	Vector2d p1 = polygon[size - 1];
	Vector2d p2 = polygon[0];
	double distance;
	for (size_t idx = 1; idx < size; idx++) {
		distance = getDistanceToLine(polygon[idx - 1], polygon[idx], point);
		if (distance < minDistance) {
			minDistance = distance;
			p1 = polygon[idx - 1];
			p2 = polygon[idx];
		}
	}

	return Line2D(p1, p2);
}

const double Geometry::getAvgDistance(const std::vector<Vector2d> &polygon1, const std::vector<Vector2d> &polygon2)
{
	if (polygon1.size() == 0 || polygon2.size() == 0) {
		return 0;
	} else if (polygon1.size() == 1) {
		getDistance(polygon2, polygon1[0]);
	} else if (polygon2.size() == 1) {
		getDistance(polygon1, polygon2[0]);
	}

	double distanceSum = 0;
	for (Vector2d p : polygon1) {
		distanceSum += getDistance(polygon2, p);
	}

	return distanceSum / polygon1.size();
}

const double Geometry::getArea(const std::vector<Vector2d> &polygon)
{
	size_t size = polygon.size();
	if (size <= 2) {
		return 0;
	}

	double sum = 0;
	// Counter clockwise order is important!
	size_t idx = size - 1;
	sum += polygon[0](0) * polygon[size - 1](1);
	sum -= polygon[0](1) * polygon[size - 1](0);
	for (; idx > 0; idx--) {
		sum += polygon[idx](0) * polygon[idx - 1](1);
		sum -= polygon[idx](1) * polygon[idx - 1](0);
	}

	return sum / 2;
}

double Geometry::constructDoubleArc(
		const Pose2D &start, const Pose2D &end, const double &radius, std::vector<LineSegment> &result)
{
	Vector2d p1 = Vector2d(start * Vector2d(0, radius));
	Vector2d p2 = Vector2d(end * Vector2d(0, -radius));

	double absRadius = std::fabs(radius);
	Circle2D c1 = Circle2D(p1, absRadius);
	Circle2D c2 = Circle2D(p2, absRadius);

	Angle angle1Start = Angle::to(start.getPosition() - p1);
	Angle angle1End = Angle::to(p2 - p1);
	Angle angle2Start = angle1End.opposite();
	Angle angle2End = Angle::to(end.getPosition() - p2);

	bool clockwise = radius < 0;
	double length = 0;

	// Construct the first segment if its spanning angle is bigger than 0.0001 deg
	if (std::fabs((angle1Start - angle1End).deg()) > 0.0001) {
		LineSegment seg(c1, angle1Start, angle1End, clockwise);
		result.push_back(seg);
		length += seg.getLength();
	}
	// Construct the second segment if its spanning angle is bigger than 0.0001 deg
	if (std::fabs((angle2Start - angle2End).deg()) > 0.0001) {
		LineSegment seg(c2, angle2Start, angle2End, !clockwise);
		result.push_back(seg);
		length += seg.getLength();
	}

	return length;
}
