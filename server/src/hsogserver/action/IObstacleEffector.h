#pragma once

#include "IEffector.h"
#include <boost/smart_ptr.hpp>

namespace taco
{
class IObstacleEffector : public virtual IEffector
{
  public:
	typedef boost::shared_ptr<IObstacleEffector> Ptr;
	typedef boost::shared_ptr<const IObstacleEffector> ConstPtr;

	virtual ~IObstacleEffector(){};

	virtual void setPosX(const double &posX) = 0;
	virtual void setPosY(const double &posY) = 0;

	virtual double getPosX() const = 0;
	virtual double getPosY() const = 0;
};
}