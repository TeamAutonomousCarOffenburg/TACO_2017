#pragma once

#include "IEffector.h"
#include <boost/smart_ptr.hpp>

namespace taco
{
class IRoadSignEffector : public virtual IEffector
{
  public:
	typedef boost::shared_ptr<IRoadSignEffector> Ptr;
	typedef boost::shared_ptr<const IRoadSignEffector> ConstPtr;

	virtual ~IRoadSignEffector(){};

	virtual void setId(const int &id) = 0;
	virtual void setPosX(const double &posX) = 0;
	virtual void setPosY(const double &posY) = 0;
	virtual void setAngle(const double &angle) = 0;

	virtual int getId() const = 0;
	virtual double getPosX() const = 0;
	virtual double getPosY() const = 0;
	virtual double getAngle() const = 0;
};
}