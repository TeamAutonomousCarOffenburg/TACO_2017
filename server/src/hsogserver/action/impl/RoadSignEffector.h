#pragma once

#include "Effector.h"
#include "action/IRoadSignEffector.h"

namespace taco
{
class RoadSignEffector : public Effector, public virtual IRoadSignEffector
{
  public:
	typedef boost::shared_ptr<RoadSignEffector> Ptr;
	typedef boost::shared_ptr<const RoadSignEffector> ConstPtr;

	RoadSignEffector(const std::string &name) : Effector(name){};
	virtual ~RoadSignEffector(){};

	virtual void setId(const int &id)
	{
		_id = id;
	}

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

	virtual int getId() const
	{
		return _id;
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
	int _id = 0;
	double _posX = 0;
	double _posY = 0;
	double _angle = 0;
};
}