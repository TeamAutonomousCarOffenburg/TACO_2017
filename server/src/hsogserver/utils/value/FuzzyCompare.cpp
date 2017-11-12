#include "FuzzyCompare.h"

using namespace taco;
using namespace Eigen;

template <typename T> bool FuzzyCompare::eq(const T &lhs, const T &rhs, const T &range)
{
	if (lhs > rhs) {
		return lhs - range <= rhs;
	}

	return rhs - range <= lhs;
}

template <typename T> bool FuzzyCompare::gte(const T &lhs, const T &rhs, const T &range)
{
	return lhs >= (rhs - range);
}

template <typename T> bool FuzzyCompare::lte(const T &lhs, const T &rhs, const T &range)
{
	return lhs <= (rhs + range);
}

template <typename T, int Size>
bool eq(const Eigen::Matrix<T, Size, 1> &lhs, const Eigen::Matrix<T, Size, 1> &rhs, const T &range)
{
	for (int i = 0; i < Size; i++) {
		if (!eq(lhs(i), rhs(i), range)) {
			return false;
		}
	}

	return true;
}

template <typename T, int Size>
bool gte(const Eigen::Matrix<T, Size, 1> &lhs, const Eigen::Matrix<T, Size, 1> &rhs, const T &range)
{
	for (int i = 0; i < Size; i++) {
		if (!gte(lhs(i), rhs(i), range)) {
			return false;
		}
	}

	return true;
}

template <typename T, int Size>
bool lte(const Eigen::Matrix<T, Size, 1> &lhs, const Eigen::Matrix<T, Size, 1> &rhs, const T &range)
{
	for (int i = 0; i < Size; i++) {
		if (!lte(lhs(i), rhs(i), range)) {
			return false;
		}
	}

	return true;
}

/**
 * Returns a linear interpolation between the two passed points at the passed
 * x coordinate. Depending on the ascending parameter the function grows
 * (true) from 0 to 1 otherwise it decreases from 1 to 0. Outside the range
 * specified by x0 and x1 the values are 0 (left, true or right, false) or 1
 * respectively (left, false or right, true.
 * @param x0 lower x value
 * @param x1 higher x value
 * @param ascending true if the lower x value represents 0 false if it
 *        represents 1
 * @param x the x value for which to get the fuzzy value
 * @return a fuzzy value that is linearly interpolated between the two passed
 *         values
 */
double FuzzyCompare::getLinearFuzzyValue(double x0, double x1, bool ascending, double x)
{
	if (x0 > x1) {
		double temp = x0;
		x0 = x1;
		x1 = temp;
	}

	if (ascending) {
		if (x <= x0) {
			return 0.0;
		}
		if (x >= x1) {
			return 1.0;
		}
		return FuzzyCompare::linearInterpolation(x0, 0, x1, 1, x);
	}

	if (x <= x0) {
		return 1.0;
	}

	if (x >= x1) {
		return 0.0;
	}

	return FuzzyCompare::linearInterpolation(x0, 1, x1, 0, x);
}

/**
 * Returns the linear interpolation (y value) of the passed points (x0,y0)
 * and (x1,y1) at the passed coordinate x
 * @param x0 x coordinate of the first point
 * @param y0 y coordinate of the first point
 * @param x1 x coordinate of the second point
 * @param y1 y coordinate of the second point
 * @param x x coordinate of the point of interest
 * @return the y coordinate of position x as linear interpolation of the two
 *         passed points
 */
double FuzzyCompare::linearInterpolation(double x0, double y0, double x1, double y1, double x)
{
	return y0 + (y1 - y0) / (x1 - x0) * (x - x0);
}
