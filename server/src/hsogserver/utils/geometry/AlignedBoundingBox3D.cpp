#include "AlignedBoundingBox3D.h"

using namespace taco;

AlignedBoundingBox3D::AlignedBoundingBox3D(
		const Eigen::Vector3d &origin, const Eigen::Vector3d &minPoint, const Eigen::Vector3d &maxPoint)
	: _origin(origin), _minPoint(minPoint), _maxPoint(maxPoint)
{
}

AlignedBoundingBox3D::AlignedBoundingBox3D(
		const Eigen::Vector3d &origin, const Eigen::Vector3d &minPoint, const Eigen::Vector3d &maxPoint, uint score)
	: _origin(origin), _minPoint(minPoint), _maxPoint(maxPoint), _score(score)
{
	if (_score > _scoreLimit)
		_score = _scoreLimit;
}
AlignedBoundingBox3D::~AlignedBoundingBox3D()
{
}

const Eigen::Vector3d AlignedBoundingBox3D::origin() const
{
	return _origin;
}

const Eigen::Vector3d AlignedBoundingBox3D::midpoint() const
{
	return Eigen::Vector3d((_maxPoint - _minPoint) + _origin);
}

const Eigen::Vector3d AlignedBoundingBox3D::maxPoint() const
{
	return _maxPoint;
}

const Eigen::Vector3d AlignedBoundingBox3D::minPoint() const
{
	return _minPoint;
}

const uint AlignedBoundingBox3D::score() const
{
	return _score;
}

const AlignedBoundingBox3D &AlignedBoundingBox3D::operator++()
{
	if (_score < _scoreLimit)
		_score = _score + 1;
	return *this;
}

const AlignedBoundingBox3D &AlignedBoundingBox3D::operator--()
{
	if (_score > 0)
		_score--;
	return *this;
}

bool AlignedBoundingBox3D::operator==(const AlignedBoundingBox3D &other) const
{
	Eigen::Vector3d otherMin = other.minPoint();
	Eigen::Vector3d otherMax = other.maxPoint();
	if ((otherMin(0) > _minPoint(0) && otherMin(1) > _minPoint(1) && otherMin(2) > _minPoint(2) &&
				otherMin(0) < _maxPoint(0) && otherMin(1) < _maxPoint(1) && otherMin(2) < _maxPoint(2)) ||
			(otherMax(0) > _minPoint(0) && otherMax(1) > _minPoint(1) && otherMax(2) > _minPoint(2) &&
					otherMax(0) < _maxPoint(0) && otherMax(1) < _maxPoint(1) && otherMax(2) < _maxPoint(2))) {
		return true;
	} else {
		return false;
	}
}

const AlignedBoundingBox3D AlignedBoundingBox3D::operator+(const AlignedBoundingBox3D &other) const
{
	Eigen::Vector3d resultMin, resultMax;
	Eigen::Vector3d offset = other.origin() - _origin;
	Eigen::Vector3d otherMinT = other.minPoint() + offset;
	Eigen::Vector3d otherMaxT = other.maxPoint() + offset;
	double minX;
	if (otherMinT(0) > _minPoint(0))
		minX = otherMinT(0);
	else
		minX = _minPoint(0);
	double minY;
	if (otherMinT(1) > _minPoint(1))
		minY = otherMinT(1);
	else
		minY = _minPoint(1);
	double minZ;
	if (otherMinT(2) > _minPoint(2))
		minZ = otherMinT(2);
	else
		minZ = _minPoint(2);
	resultMin = Eigen::Vector3d(minX, minY, minZ);
	double maxX;
	if (otherMaxT(0) > _maxPoint(0))
		maxX = otherMaxT(0);
	else
		maxX = _maxPoint(0);
	double maxY;
	if (otherMaxT(1) > _maxPoint(1))
		maxY = otherMaxT(1);
	else
		maxY = _maxPoint(1);
	double maxZ;
	if (otherMaxT(2) > _maxPoint(2))
		maxZ = otherMaxT(2);
	else
		maxZ = _maxPoint(2);
	resultMax = Eigen::Vector3d(maxX, maxY, maxZ);

	return AlignedBoundingBox3D(_origin, resultMin, resultMax, _score + other.score());
}

const uint AlignedBoundingBox3D::operator+(const uint &other) const
{
	uint newScore = other + _score;
	if (newScore > _scoreLimit)
		return _scoreLimit;
	else
		return newScore;
}

const uint AlignedBoundingBox3D::operator-(const uint &other) const
{
	if (other > _score)
		return 0;
	else
		return _score - other;
}

const void AlignedBoundingBox3D::operator+=(const uint &other)
{
	_score += other;
}

const void AlignedBoundingBox3D::operator-=(const uint &other)
{
	_score -= other;
}

bool AlignedBoundingBox3D::operator!=(const uint &other) const
{
	return _score != other;
}

bool AlignedBoundingBox3D::operator<(const uint &other) const
{
	return _score < other;
}

bool AlignedBoundingBox3D::operator<=(const uint &other) const
{
	return _score <= other;
}

bool AlignedBoundingBox3D::operator==(const uint &other) const
{
	return _score == other;
}

bool AlignedBoundingBox3D::operator>(const uint &other) const
{
	return _score > other;
}

bool AlignedBoundingBox3D::operator>=(const uint &other) const
{
	return _score >= other;
}

void AlignedBoundingBox3D::setScoreLimit(const uint &limit)
{
	_scoreLimit = limit;
}
