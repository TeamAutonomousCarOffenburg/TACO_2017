#pragma once

#include "IEffector.h"

#include <boost/smart_ptr.hpp>

namespace taco
{
class IManeuverStatusEffector : public virtual IEffector
{
  public:
	typedef boost::shared_ptr<IManeuverStatusEffector> Ptr;
	typedef boost::shared_ptr<const IManeuverStatusEffector> ConstPtr;

	virtual ~IManeuverStatusEffector(){};

	virtual void setManeuverId(const int &id) = 0;
	virtual void setStatus(const int &status) = 0;

	virtual int getStatus() const = 0;
	virtual int getManeuverId() const = 0;
};
}
