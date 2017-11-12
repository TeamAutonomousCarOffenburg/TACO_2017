#pragma once

#include "Angle.h"
#include "Circle2D.h"
#include "Line2D.h"
#include "LineSegment.h"
#include "Polygon.h"
#include "Pose2D.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>

namespace taco
{
/** \brief Utility class for geometrical calculations in 2D and 3D space.
 *
 * The Geometry class provides useful calculations to vectors, lines, circles, etc. in 2D and 3D space.
 *
 * \author Stefan Glaser, Peter Walden
 */
class Geometry
{
  public:
	/** Retireve the angle delta (altitude/polar-/vertical-angle) in the x,y plane. */
	static const Angle getDelta(const Eigen::Vector3d &vector);

	/** Calculate the destance between two points. */
	static const double getDistance(const double &p1x, const double &p1y, const double &p2x, const double &p2y);

	/** Calculate the distance between two points. */
	template <typename T, int Size>
	static const double getDistance(const Eigen::Matrix<T, Size, 1> &v1, const Eigen::Matrix<T, Size, 1> &v2)
	{
		return getNorm<T, Size>(v2 - v1);
	};

	/** Calculate the norm (length) of the given vector. */
	template <typename T, int Size> static const double getNorm(const Eigen::Matrix<T, Size, 1> &vector)
	{
		// Fetch some special cases for (potentially) better runtime
		switch (Size) {
		case 1:
			return std::abs(vector(0));
		case 2:
			return std::sqrt(vector(0) * vector(0) + vector(1) * vector(1));
		case 3:
			return std::sqrt(vector(0) * vector(0) + vector(1) * vector(1) + vector(2) * vector(2));
		case 4:
			return std::sqrt(
					vector(0) * vector(0) + vector(1) * vector(1) + vector(2) * vector(2) + vector(3) * vector(3));
		default:
			// Generic length calculation
			double squaredSum = 0;

			for (int i = 0; i < Size; i++) {
				squaredSum += vector(i) * vector(i);
			}

			return std::sqrt(squaredSum);
		};
	};

	/** Calculate the average of the given points. */
	template <typename T, int Size>
	static const Eigen::Matrix<T, Size, 1> avg(const std::vector<Eigen::Matrix<T, Size, 1>> &points)
	{
		size_t pSize = points.size();
		if (pSize == 0) {
			return Eigen::Matrix<T, Size, 1>();
		}

		T sum[Size];

		for (int i = 0; i < Size; i++) {
			sum[i] = 0;
			for (Eigen::Matrix<T, Size, 1> p : points) {
				sum[i] += p(i);
			}
			sum[i] = sum[i] / pSize;
		}

		return Eigen::Matrix<T, Size, 1>(sum);
	};

	/** Calculate the weighted average of the two given points. */
	template <typename T, int Size>
	static const Eigen::Matrix<T, Size, 1> avg(const Eigen::Matrix<T, Size, 1> &p1, const Eigen::Matrix<T, Size, 1> &p2,
			const T &weight1 = 1, const T &weight2 = 1)
	{
		T avg[Size];

		for (int i = 0; i < Size; i++) {
			avg[i] = (p1(i) * weight1 + p2(i) * weight2) / (weight1 + weight2);
		}

		return Eigen::Matrix<T, Size, 1>(avg);
	};

	/** Calculate the bounding box of an point set.
	 * \returns the pair {min, max} representing the minimum and maximum values for each dimension
	 */
	template <typename T, int Size>
	static const std::pair<Eigen::Matrix<T, Size, 1>, Eigen::Matrix<T, Size, 1>> getBounds(
			const std::vector<Eigen::Matrix<T, Size, 1>> &points)
	{
		Eigen::Matrix<T, Size, 1> min;
		Eigen::Matrix<T, Size, 1> max;

		for (Eigen::Matrix<T, Size, 1> p : points) {
			for (int i = 0; i < Size; i++) {
				if (p(i) < min(i)) {
					min(i) = p(i);
				}
				if (p(i) > max(i)) {
					max(i) = p(i);
				}
			}
		}

		return std::pair<Eigen::Matrix<T, Size, 1>, Eigen::Matrix<T, Size, 1>>(min, max);
	}

	/** Retrieve the point with the minimum value in the specified dimension.
	 * \returns the point with the minimum value in the specified dimension
	 */
	template <typename T, int Size>
	static const Eigen::Matrix<T, Size, 1> min(
			const std::vector<Eigen::Matrix<T, Size, 1>> &points, const unsigned int &dimension)
	{
		return minmax(points, dimension, true);
	}

	/** Retrieve the point with the minimum value in the specified dimension.
	 * \returns the point with the minimum value in the specified dimension
	 */
	template <typename T, int Size>
	static const Eigen::Matrix<T, Size, 1> max(
			const std::vector<Eigen::Matrix<T, Size, 1>> &points, const unsigned int &dimension)
	{
		return minmax(points, dimension, false);
	}

	/** Retrieve the point with the minimum/maximum value in the specified dimension.
	 * \returns the point with the minimum/maximum value in the specified dimension
	 */
	template <typename T, int Size>
	static const Eigen::Matrix<T, Size, 1> minmax(
			const std::vector<Eigen::Matrix<T, Size, 1>> &points, const unsigned int &dimension, const bool &min)
	{
		if (points.size() == 0) {
			return Eigen::Matrix<T, Size, 1>();
		}

		int dim = dimension;
		if (dim >= Size) {
			dim %= Size;
		}

		size_t minIdx = 0;
		if (min) {
			for (size_t i = 1; i < points.size(); i++) {
				if (points[i](dim) < points[minIdx](dim)) {
					minIdx = i;
				}
			}
		} else {
			for (size_t i = 1; i < points.size(); i++) {
				if (points[i](dim) > points[minIdx](dim)) {
					minIdx = i;
				}
			}
		}

		return points[minIdx];
	}

	static Angle getHorizontalAngle(const Eigen::Quaterniond &orientation);

	/** Check if two lines are parallel. */
	static const bool checkParallel(const Line2D &line1, const Line2D &line2);

	/** Calculate the intersection point of the two lines.
	 *
	 * The intersection point (if existing) is stored in the result vector.
	 *
	 * \param line1 - the first line
	 * \param line2 - the second line
	 * \param result - the vector to store the intersection point if existing
	 * \returns true, if a unique intersection point exists, false if both lines are parallel
	 */
	static const bool intersectionPoint(const Line2D &line1, const Line2D &line2, Eigen::Vector2d &result);

	/** Caclulate the intersection points of the given lince and circle.
	 *
	 * The intersection point (if existing) is stored in the result vector.
	 *
	 * \param line - the line
	 * \param circle - the circle
	 * \param result - the vector to store the intersection points if existing
	 */
	static void intersectionPoints(const Line2D &line, const Circle2D &circle, std::vector<Eigen::Vector2d> &result);

	/** Caclulate the intersection points of the two circles.
	 *
	 * The intersection points (if existing) are stored in the result vector.
	 *
	 * \param circle1 - the first circle
	 * \param circle2 - the second circle
	 * \param result - the vector to store the intersection points if existing
	 */
	static void intersectionPoints(
			const Circle2D &circle1, const Circle2D &circle2, std::vector<Eigen::Vector2d> &result);

	/** Calculate the common tangents between two circles.
	 *
	 * \param circle1 - the first circle
	 * \param circle2 - the second circle
	 * \param result - the vector to store the common tangents if existing
	 */
	static void commonTangents(const Circle2D &circle1, const Circle2D &circle2, std::vector<Line2D> &result);

	/** Calculate the slope on given point.
	 *
	 * \param circle - the given circle
	 * \param point - the coordinate on which the slope should be calculated for
	 * \param result - the resulting angle of the tangent to the x-axias
	 *
	 */
	static void slope(const Circle2D &circle, const Eigen::Vector2d &point, Angle &result);

	/** Calculate the touching point of a line on the x axias (y = 0) and
	 * a circle with a tangential point representet by circleSideTransformed.
	 *
	 * \param circleSideTransformed - tangential point of the circle
	 *
	 * \return offset on x axias of touching point from line with circle (y = 0)
	 */
	static double getTouchingPoint(const Pose2D &circleSideTransformed);

	static Pose2D calculateArcPose(const double &length, const Angle &exitAngle);

	static Pose2D calculateArcPose(const double &length, const double &exitAngle);

	/** Interpolate a bunch of intermediate poses on an arc.
	 * Only intermediate poses are extracted, the begin and end pose of the arc are not included!
	 */
	static void interpolateArcPoses(const double &length, const double &exitAngle, const size_t &howMany,
			std::vector<Pose2D> &result, const Pose2D &startPose = Pose2D());

	static double constructDoubleArc(const Pose2D &start, const Pose2D &end, std::vector<LineSegment> &result);

	static const double getDistanceToLine(const Line2D &line, const Eigen::Vector2d &point);

	static const double getDistanceToLine(
			const Eigen::Vector2d &lineStart, const Eigen::Vector2d &lineEnd, const Eigen::Vector2d &point);

	/**  Checks if the passed point is inside the passed polygon
	 * \param polygon corner points of the polygon in clockwise direction
	 * \param position the point to check
	 * \returns true if the passed point is inside the polygon, false otherwise
	 */
	static const bool isInsidePolygon(const std::vector<Eigen::Vector2d> &polygon, const Eigen::Vector2d &point);

	static const double getDistance(const std::vector<Eigen::Vector2d> &polygon, const Eigen::Vector2d &point);

	static const Line2D getClosestPolyLine(const std::vector<Eigen::Vector2d> &polygon, const Eigen::Vector2d &point);

	static const double getAvgDistance(
			const std::vector<Eigen::Vector2d> &polygon1, const std::vector<Eigen::Vector2d> &polygon2);

	static const double getArea(const std::vector<Eigen::Vector2d> &polygon);

  private:
	static double constructDoubleArc(
			const Pose2D &start, const Pose2D &end, const double &radius, std::vector<LineSegment> &result);
};
}
