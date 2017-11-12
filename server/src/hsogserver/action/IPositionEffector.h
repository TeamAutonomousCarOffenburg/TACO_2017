#pragma once

#include "IEffector.h"
#include <boost/smart_ptr.hpp>

namespace taco
{
class IPositionEffector : public virtual IEffector
{
  public:
	typedef boost::shared_ptr<IPositionEffector> Ptr;
	typedef boost::shared_ptr<const IPositionEffector> ConstPtr;

	virtual ~IPositionEffector(){};

	virtual void setPosX(const double &posX) = 0;
	virtual void setPosY(const double &posY) = 0;
	virtual void setAngle(const double &angle) = 0;

	virtual double getPosX() const = 0;
	virtual double getPosY() const = 0;
	virtual double getAngle() const = 0;
};
}