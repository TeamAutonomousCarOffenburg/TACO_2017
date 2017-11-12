#include "Angle.h"

#include <cmath>

using namespace taco;
using namespace Eigen;

Angle Angle::ZERO(0);
Angle Angle::EPSILON(0.0000001);
Angle Angle::DEG_180(M_PI);
Angle Angle::DEG_90(M_PI / 2);
Angle Angle::DEG_N90(-M_PI / 2);

// Initialize static members
double Angle::radPerDeg = M_PI / 180.0;

Angle::Angle() : _radAngle(0)
{
}

Angle::Angle(const double &rad) : _radAngle(rad)
{
	normalize(_radAngle);
}

Angle::Angle(const Angle &other) : _radAngle(other._radAngle)
{
}

Angle::~Angle()
{
}

const double &Angle::rad() const
{
	return _radAngle;
}

const double Angle::absRad() const
{
	if (_radAngle < 0) {
		return -_radAngle;
	}

	return _radAngle;
}

const double Angle::positiveRad() const
{
	if (_radAngle < 0) {
		return (2 * M_PI + _radAngle);
	}
	return _radAngle;
}

const double Angle::deg() const
{
	return _radAngle / radPerDeg;
}

const double Angle::absDeg() const
{
	return std::fabs(_radAngle / radPerDeg);
}

const double Angle::sin() const
{
	return std::sin(_radAngle);
}

const double Angle::cos() const
{
	return std::cos(_radAngle);
}

const double Angle::tan() const
{
	return std::tan(_radAngle);
}

Angle &Angle::operator=(const Angle &rhs)
{
	_radAngle = rhs._radAngle;
	return *this;
}

Angle &Angle::operator+=(const Angle &rhs)
{
	_radAngle += rhs._radAngle;
	normalize(_radAngle);
	return *this;
}

Angle &Angle::operator-=(const Angle &rhs)
{
	_radAngle -= rhs._radAngle;
	normalize(_radAngle);
	return *this;
}

void Angle::setRadians(const double &rad)
{
	_radAngle = rad;
	normalize(_radAngle);
}

void Angle::setDegrees(const double &deg)
{
	_radAngle = deg * radPerDeg;
	normalize(_radAngle);
}

const Angle Angle::operator+(const Angle &rhs) const
{
	return Angle(*this) += rhs;
}

const Angle Angle::operator-(const Angle &rhs) const
{
	return Angle(*this) -= rhs;
}

const Vector2d Angle::operator*(const Vector2d &vec) const
{
	Vector2d res;
	rotate(vec, _radAngle, res);
	return res;
}

const Vector2d Angle::operator/(const Vector2d &vec) const
{
	Vector2d res;
	rotate(vec, -_radAngle, res);
	return res;
}

bool Angle::operator==(const Angle &rhs) const
{
	return _radAngle == rhs._radAngle;
}

bool Angle::operator!=(const Angle &rhs) const
{
	return !(*this == rhs);
}

bool Angle::operator>(const Angle &rhs) const
{
	return _radAngle > rhs._radAngle;
}

bool Angle::operator>=(const Angle &rhs) const
{
	return _radAngle >= rhs._radAngle;
}

bool Angle::operator<(const Angle &rhs) const
{
	return _radAngle < rhs._radAngle;
}

bool Angle::operator<=(const Angle &rhs) const
{
	return _radAngle <= rhs._radAngle;
}

bool Angle::eq(const Angle &rhs, const Angle &range) const
{
	double distance = std::fabs(_radAngle - rhs._radAngle);
	if (distance > M_PI) {
		distance = 2 * M_PI - distance;
	}
	return distance <= std::fabs(range._radAngle);
}

bool Angle::gte(const Angle &rhs, const Angle &range) const
{
	return _radAngle > rhs._radAngle || this->eq(rhs, range);
}

bool Angle::lte(const Angle &rhs, const Angle &range) const
{
	return _radAngle < rhs._radAngle || this->eq(rhs, range);
}

const Angle Angle::operator!() const
{
	return Angle(-_radAngle);
}

const Angle Angle::negate() const
{
	return Angle(-_radAngle);
}

const Angle Angle::opposite() const
{
	return Angle(_radAngle + M_PI);
}

const Angle Angle::normal() const
{
	return Angle(_radAngle + M_PI * 0.5);
}

const bool Angle::isOppositeOf(const Angle &other) const
{
	if (_radAngle < 0) {
		return std::abs((other._radAngle - (_radAngle + M_PI))) < 0.0000001;
	} else {
		return std::abs((other._radAngle - (_radAngle - M_PI))) < 0.0000001;
	}
}

const Vector2d Angle::getVector(const double &length) const
{
	return Vector2d(std::cos(_radAngle) * length, std::sin(_radAngle) * length);
}

const double Angle::getDifferenceTo(const Angle &other, const bool &clockwise) const
{
	double diff = other._radAngle - _radAngle;

	if (clockwise && diff > 0) {
		diff -= 2 * M_PI;
	} else if (!clockwise && diff < 0) {
		diff += 2 * M_PI;
	}

	return diff;
}

const double Angle::getDistanceTo(const Angle &other) const
{
	double clockwose = getDifferenceTo(other, true);
	double counterClockwose = getDifferenceTo(other, false);

	return std::min(clockwose, counterClockwose);
}

// ========== STATIC HELPER METHODS ==========
const Angle Angle::deg(const double &deg)
{
	return Angle(deg * radPerDeg);
}

const Angle Angle::rad(const double &rad)
{
	return Angle(rad);
}

const Angle Angle::to(const Eigen::Vector2d &point)
{
	if (point(0) == 0 && point(1) == 0) {
		return Angle::Zero();
	}

	return Angle(std::atan2(point(1), point(0)));
}

const Angle Angle::to(const double &x, const double &y)
{
	if (x == 0 && y == 0) {
		return Angle::Zero();
	}

	return Angle(std::atan2(y, x));
}

void Angle::normalize(double &rad)
{
	if (rad >= 0) {
		rad += M_PI;
		rad = std::fmod(rad, 2 * M_PI);
		rad -= M_PI;
	} else {
		rad = -rad + M_PI;
		rad = std::fmod(rad, 2 * M_PI);
		rad = -(rad - M_PI);
		if (rad >= M_PI) {
			rad = -rad;
		}
	}
}

void Angle::rotate(const Vector2d &vec, const double &radAngle, Vector2d &res)
{
	double cosAngle = std::cos(radAngle);
	double sinAngle = std::sin(radAngle);

	// Calculate new position
	res(0) = cosAngle * vec(0) - sinAngle * vec(1);
	res(1) = sinAngle * vec(0) + cosAngle * vec(1);
}

double Angle::toRadians(const double &deg)
{
	return deg * radPerDeg;
};

double Angle::toDegrees(const double &rad)
{
	return rad / radPerDeg;
};

const Angle &Angle::Zero()
{
	return ZERO;
}

const Angle &Angle::Epsilon()
{
	return EPSILON;
}

const Angle &Angle::Deg_180()
{
	return DEG_180;
}

const Angle &Angle::Deg_90()
{
	return DEG_90;
}

const Angle &Angle::Deg_N90()
{
	return DEG_N90;
}

const Angle Angle::average(const Angle &a1, const Angle &a2, const double &weight1, const double &weight2)
{
	return ((weight1 * (a1.rad() + M_PI) + weight2 * (a2.rad() + M_PI)) / (weight1 + weight2)) - M_PI;
}

const Angle Angle::average(const std::vector<Angle> &angles)
{
	if (angles.size() == 0) {
		return 0;
	} else if (angles.size() == 1) {
		return angles[0];
	}

	double sum = 0;

	for (Angle a : angles) {
		sum += a.rad() + M_PI;
	}

	return (sum / angles.size()) - M_PI;
}

const Angle Angle::interpolate(const Angle &startAngle, const Angle &endAngle, const double &progress)
{
	if (progress <= 0) {
		return startAngle;
	} else if (progress >= 1) {
		return endAngle;
	}

	double diff = endAngle._radAngle - startAngle._radAngle;
	if (diff > M_PI) {
		diff -= 2 * M_PI;
	} else if (diff < -M_PI) {
		diff += 2 * M_PI;
	}

	return Angle(startAngle._radAngle + (diff * progress));
}

namespace taco
{
std::ostream &operator<<(std::ostream &os, const taco::Angle &angle)
{
	os << (angle.rad() * M_PI / 180);
	return os;
}
}
