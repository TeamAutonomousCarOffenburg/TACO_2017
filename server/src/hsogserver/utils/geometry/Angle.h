#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <iostream>
#include <vector>

namespace taco
{
/** \brief Representation of an angle in the range from -2 Pi to 2 Pi degrees.
 *
 * The Angle class represents an angle in 2-dimensional space.
 * This class takes care of limiting the represented angle within +/- 180 degrees, respectively +/- M_PI radians.
 *
 * \author Stefan Glaser
 */
class Angle
{
  public:
	/** Default constructor creating a zero angle. */
	Angle();

	/** Construct an new Angle with the given radian angle value. */
	Angle(const double &rad);

	/** Copy-Constructor, copying the angular value of the other Angle. */
	Angle(const Angle &other);

	~Angle();

	/** Retrieve the angular value in radians. */
	const double &rad() const;

	/** Retrieve the absolute angular value in radians. */
	const double absRad() const;

	/** Retrieve the angular value in positive radians in the range of 0 =< a < 2PI */
	const double positiveRad() const;

	/** Retrieve the angular value in degrees. */
	const double deg() const;

	/** Retrieve the absolute angular value in degrees. */
	const double absDeg() const;

	/** Retrieve sinus value of this angle. */
	const double sin() const;

	/** Retrieve cosinus value of this angle. */
	const double cos() const;

	/** Retrieve tangens value of this angle. */
	const double tan() const;

	Angle &operator=(const Angle &rhs);
	Angle &operator+=(const Angle &rhs);
	Angle &operator-=(const Angle &rhs);

	/** Set the angular value in radians. */
	void setRadians(const double &rad);

	/** Set the angular value in degrees. */
	void setDegrees(const double &deg);

	const Angle operator+(const Angle &rhs) const;
	const Angle operator-(const Angle &rhs) const;

	/** Apply this angle to the given vector.
	 * Transform the given 2-dimensional vector about this angle.
	 * \param vec - the vector to transform
	 * \returns a transformed version of the given vector
	 */
	const Eigen::Vector2d operator*(const Eigen::Vector2d &vec) const;

	/** Apply this angle INVERSE to the given vector.
	 * Inverse-transform the given 2-dimensional vector about this angle.
	 * \param vec - the vector to transform
	 * \returns a inverse-transformed version of the given vector
	 */
	const Eigen::Vector2d operator/(const Eigen::Vector2d &vec) const;

	bool operator==(const Angle &rhs) const;
	bool operator!=(const Angle &rhs) const;
	bool operator>(const Angle &rhs) const;
	bool operator>=(const Angle &rhs) const;
	bool operator<(const Angle &rhs) const;
	bool operator<=(const Angle &rhs) const;

	/** Fuzzy version of "this == rhs". */
	bool eq(const Angle &rhs, const Angle &range = Angle::Epsilon()) const;

	/** Fuzzy version of "this >= rhs". */
	bool gte(const Angle &rhs, const Angle &range = Angle::Epsilon()) const;

	/** Fuzzy version of "this <= rhs". */
	bool lte(const Angle &rhs, const Angle &range = Angle::Epsilon()) const;

	/** Retrieve a negative copy of this angle. (see method negate) */
	const Angle operator!() const;

	/** Retrieve a negative copy of this angle. (see operator: "!") */
	const Angle negate() const;

	/** Retrieve a copy of this angle pointing in the opposite direction (the result of this + 180 deg). */
	const Angle opposite() const;

	/** Retrieve a copy of this angle rotated + Pi/2, representing the angle of the normal */
	const Angle normal() const;

	const bool isOppositeOf(const Angle &other) const;

	/** Get a vector of length length in the direction of this angle. */
	const Eigen::Vector2d getVector(const double &length = 1) const;

	/** Retrieve the angular distance from this angle to the other angle in the given direction. */
	const double getDifferenceTo(const Angle &other, const bool &clockwise = false) const;

	/** Retrieve the minimal distance (clockwise or counter ckockwise) from this angle to the other angle. */
	const double getDistanceTo(const Angle &other) const;

	//   friend std::ostream& operator<<(std::ostream& os, const Angle& angle);

	// ========== STATIC HELPER METHODS ==========
	/** Create a new Angle using the given radian angle value. */
	static const Angle rad(const double &rad);

	/** Create a new Angle using the given degrees angle value. */
	static const Angle deg(const double &deg);

	/** Create a new Angle to the given point. */
	static const Angle to(const Eigen::Vector2d &point);

	/** Create a new Angle to the given point. */
	static const Angle to(const double &x, const double &y);

	/** Transform the given value from degrees to radian. */
	static double toRadians(const double &deg);

	/** Transform the given value from radian to degrees. */
	static double toDegrees(const double &rad);

	/** Normalize the given radian angle value (restrict to range +/- M_PI radians). */
	static void normalize(double &rad);

	/** Rotate the given vector vec about the radian angle and store the result in res.
	 * \param vec - the vector to transform
	 * \param radAngle - the radian angle to rotate
	 * \param res - the resulting vector
	 */
	static void rotate(const Eigen::Vector2d &vec, const double &radAngle, Eigen::Vector2d &res);

	static const Angle &Zero();
	static const Angle &Epsilon();
	static const Angle &Deg_180();
	static const Angle &Deg_90();
	static const Angle &Deg_N90();

	static const Angle average(const Angle &a1, const Angle &a2, const double &weight1 = 1, const double &weight2 = 1);

	static const Angle average(const std::vector<Angle> &angles);

	static const Angle interpolate(const Angle &startAngle, const Angle &endAngle, const double &progress);

  private:
	static Angle ZERO;
	static Angle EPSILON;
	static Angle DEG_180;
	static Angle DEG_90;
	static Angle DEG_N90;

	/** The angle in radians. */
	double _radAngle;

	/** Factor / Divisor for transforming radians to degrees and vice versa. */
	static double radPerDeg;
};

std::ostream &operator<<(std::ostream &os, const Angle &angle);
}
