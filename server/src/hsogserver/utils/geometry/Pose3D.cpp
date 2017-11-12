#include "Pose3D.h"

using namespace taco;
using namespace Eigen;

Pose3D::Pose3D()
{
}

Pose3D::Pose3D(const Pose3D &other) : _position(other._position), _orientation(other._orientation)
{
}

Pose3D::Pose3D(const Vector3d &position) : _position(position)
{
	_orientation.setIdentity();
}

Pose3D::Pose3D(const double &x, const double &y, const double &z) : _position(x, y, z)
{
	_orientation.setIdentity();
}

Pose3D::Pose3D(const Quaterniond &orientation) : _position(0, 0, 0), _orientation(orientation)
{
}

Pose3D::Pose3D(const Vector3d &position, const Quaterniond &orientation)
	: _position(position), _orientation(orientation)
{
}

Pose3D::Pose3D(const double &x, const double &y, const double &z, const Quaterniond &orientation)
	: _position(x, y, z), _orientation(orientation)
{
}

Pose3D::Pose3D(const AngleAxisd &orientation) : _position(0, 0, 0), _orientation(orientation)
{
}

Pose3D::Pose3D(const Vector3d &position, const AngleAxisd &orientation) : _position(position), _orientation(orientation)
{
}

Pose3D::Pose3D(const double &x, const double &y, const double &z, const AngleAxisd &orientation)
	: _position(x, y, z), _orientation(orientation)
{
}

Pose3D::~Pose3D()
{
}

double Pose3D::x() const
{
	return _position(0);
}

double Pose3D::y() const
{
	return _position(1);
}

double Pose3D::z() const
{
	return _position(2);
}

const Vector3d &Pose3D::getPosition() const
{
	return _position;
}

const Quaterniond &Pose3D::getOrientation() const
{
	return _orientation;
}

Pose3D &Pose3D::operator=(const Pose3D &rhs)
{
	_position = rhs._position;
	_orientation = rhs._orientation;
	return *this;
}

Pose3D &Pose3D::operator*=(const Pose3D &rhs)
{
	_position += _orientation * rhs._position;
	_orientation *= rhs._orientation;
	return *this;
}

Pose3D &Pose3D::operator-=(const Pose3D &rhs)
{
	Quaterniond inverseOrientation(rhs._orientation.conjugate());
	_position = inverseOrientation * (_position - rhs._position);
	_orientation = inverseOrientation * _orientation;
	return *this;
}

Pose3D &Pose3D::operator/=(const Pose3D &rhs)
{
	Quaterniond inverseOrientation(_orientation.conjugate());
	_position = inverseOrientation * (rhs._position - _position);
	_orientation = inverseOrientation * rhs._orientation;
	return *this;
}

void Pose3D::setPosition(const Vector3d &position)
{
	_position = position;
}

void Pose3D::setPosition(const double &x, const double &y, const double &z)
{
	_position(0) = x;
	_position(1) = y;
	_position(2) = z;
}

void Pose3D::setOrientation(const Quaterniond &orientation)
{
	_orientation = orientation;
}

void Pose3D::setOrientation(const AngleAxisd &orientation)
{
	_orientation = orientation;
}

const Pose3D Pose3D::operator*(const Pose3D &other) const
{
	return Pose3D(*this) *= other;
}

const Pose3D Pose3D::operator-(const Pose3D &other) const
{
	return Pose3D(*this) -= other;
}

const Pose3D Pose3D::operator/(const Pose3D &other) const
{
	return Pose3D(*this) /= other;
}

const Vector3d Pose3D::operator*(const Vector3d &vec) const
{
	return _position + (_orientation * vec);
}

const Vector3d Pose3D::operator/(const Vector3d &vec) const
{
	return _orientation.conjugate() * (vec - _position);
}

const AlignedBoundingBox3D Pose3D::operator*(const AlignedBoundingBox3D &box) const
{
	const AlignedBoundingBox3D transformedBox(
			*this * box.origin(), _orientation * box.minPoint(), _orientation * box.maxPoint());
	return transformedBox;
}

const AlignedBoundingBox3D Pose3D::operator/(const AlignedBoundingBox3D &box) const
{
	Quaterniond con = _orientation.conjugate();
	const AlignedBoundingBox3D transformedBox(*this / box.origin(), con * box.minPoint(), con * box.maxPoint());
	return transformedBox;
}

bool Pose3D::operator==(const Pose3D &other) const
{
	return _position == other._position && _orientation.angularDistance<Quaterniond>(other._orientation) < 0.000001;
}

bool Pose3D::operator!=(const Pose3D &other) const
{
	return !(*this == other);
}

const Pose3D Pose3D::invert() const
{
	Quaterniond inverseOrientation = _orientation.conjugate();
	Vector3d inversePos = -1 * (inverseOrientation * _position);

	return Pose3D(inversePos, inverseOrientation);
}

namespace taco
{
std::ostream &operator<<(std::ostream &os, const Pose3D &pose)
{
	os << pose.x() << " " << pose.y() << " " << pose.z();
	return os;
}
}
