#pragma once

#include <string>
#include <vector>

namespace taco
{
class ConfigRoadSign
{
  public:
	ConfigRoadSign(const std::int16_t &id, const float &xPos, const float &yPos, const float &radius,
			const std::int16_t &direction)
	{
		_id = id;
		_xPos = xPos;
		_yPos = yPos;
		_radius = radius;
		_direction = direction;
	};

	virtual ~ConfigRoadSign(){};

	virtual const std::int16_t &getId() const
	{
		return _id;
	}

	virtual const float &getXPos() const
	{
		return _xPos;
	}
	virtual const float &getYPos() const
	{
		return _yPos;
	}
	virtual const float &getRadius() const
	{
		return _radius;
	}
	virtual const std::int16_t &getDirection() const
	{
		return _direction;
	}

  protected:
	std::int16_t _id;
	float _xPos;
	float _yPos;
	float _radius;
	std::int16_t _direction;
};
}