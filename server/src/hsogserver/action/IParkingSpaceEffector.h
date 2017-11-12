#pragma once

#include "IEffector.h"
#include <boost/smart_ptr.hpp>

namespace taco
{
class IParkingSpaceEffector : public virtual IEffector
{
  public:
	typedef boost::shared_ptr<IParkingSpaceEffector> Ptr;
	typedef boost::shared_ptr<const IParkingSpaceEffector> ConstPtr;

	virtual ~IParkingSpaceEffector(){};

	virtual void setId(const int &id) = 0;
	virtual void setPosX(const double &posX) = 0;
	virtual void setPosY(const double &posY) = 0;
	virtual void setState(const int &state) = 0;

	virtual int getId() const = 0;
	virtual double getPosX() const = 0;
	virtual double getPosY() const = 0;
	virtual int getState() const = 0;
};
}