#pragma once

#include <string>
#include <vector>

namespace taco
{
class ParkingSpace
{
  public:
	ParkingSpace(const std::int16_t &id, const float &xPos, const float &yPos, const std::int16_t &state,
			const std::int16_t &direction)
	{
		_id = id;
		_xPos = xPos;
		_yPos = yPos;
		_state = state;
		_direction = direction;
	};

	virtual ~ParkingSpace(){};

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
	virtual const int16_t &getState() const
	{
		return _state;
	}
	virtual const std::int16_t &getDirection() const
	{
		return _direction;
	}

  protected:
	std::int16_t _id;
	float _xPos;
	float _yPos;
	int16_t _state;
	std::int16_t _direction;
};
}