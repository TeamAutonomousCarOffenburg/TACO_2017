#pragma once

#include "Effector.h"
#include "action/IPositionEffector.h"

namespace taco
{
class PositionEffector : public Effector, public virtual IPositionEffector
{
  public:
	typedef boost::shared_ptr<PositionEffector> Ptr;
	typedef boost::shared_ptr<const PositionEffector> ConstPtr;

	PositionEffector(const std::string &name) : Effector(name){};
	virtual ~PositionEffector(){};

	virtual void setPosX(const double &posX)
	{
		_posX = posX;
	}

	virtual void setPosY(const double &posY)
	{
		_posY = posY;
	}

	virtual void setAngle(const double &angle)
	{
		_angle = angle;
	}

	virtual double getPosX() const
	{
		return _posX;
	}

	virtual double getPosY() const
	{
		return _posY;
	}

	virtual double getAngle() const
	{
		return _angle;
	}

  protected:
	double _posX = 0;
	double _posY = 0;
	double _angle = 0;
};
}