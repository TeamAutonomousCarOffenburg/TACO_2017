#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace taco
{
/** Util class providing fuzzy (ranged) comparisons of different values.
 *
 * \author Stefan Glaser
 */
class FuzzyCompare
{
  public:
	/** Check: rhs - range <= lhs <= rhs + range */
	template <typename T> static bool eq(const T &lhs, const T &rhs, const T &range);

	/** Check: lhs >= rhs - range */
	template <typename T> static bool gte(const T &lhs, const T &rhs, const T &range);

	/** Check: lhs <= rhs + range */
	template <typename T> static bool lte(const T &lhs, const T &rhs, const T &range);

	/** Check for x, y: rhs - range <= lhs <= rhs + range */
	template <typename T, int Size>
	static bool eq(const Eigen::Matrix<T, Size, 1> &lhs, const Eigen::Matrix<T, Size, 1> &rhs, const T &range);

	/** Check for x, y: lhs >= rhs - range */
	template <typename T, int Size>
	static bool gte(const Eigen::Matrix<T, Size, 1> &lhs, const Eigen::Matrix<T, Size, 1> &rhs, const T &range);

	/** Check for x, y: lhs <= rhs + range */
	template <typename T, int Size>
	static bool lte(const Eigen::Matrix<T, Size, 1> &lhs, const Eigen::Matrix<T, Size, 1> &rhs, const T &range);

	static double getLinearFuzzyValue(double x0, double x1, bool ascending, double x);

	static double linearInterpolation(double x0, double y0, double x1, double y1, double x);
};
}
