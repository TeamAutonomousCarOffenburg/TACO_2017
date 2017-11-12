#pragma once

#include "Effector.h"
#include "action/IObstacleEffector.h"

namespace taco
{
class ObstacleEffector : public Effector, public virtual IObstacleEffector
{
  public:
	typedef boost::shared_ptr<ObstacleEffector> Ptr;
	typedef boost::shared_ptr<const ObstacleEffector> ConstPtr;

	ObstacleEffector(const std::string &name) : Effector(name){};
	virtual ~ObstacleEffector(){};

	virtual void setPosX(const double &posX)
	{
		_posX = posX;
	}

	virtual void setPosY(const double &posY)
	{
		_posY = posY;
	}

	virtual double getPosX() const
	{
		return _posX;
	}

	virtual double getPosY() const
	{
		return _posY;
	}

  protected:
	double _posX = 0;
	double _posY = 0;
};
}
