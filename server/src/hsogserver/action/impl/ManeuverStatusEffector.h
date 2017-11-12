#pragma once

#include "Effector.h"
#include "action/IManeuverStatusEffector.h"

namespace taco
{
class ManeuverStatusEffector : public Effector, public virtual IManeuverStatusEffector
{
  public:
	typedef boost::shared_ptr<ManeuverStatusEffector> Ptr;
	typedef boost::shared_ptr<const ManeuverStatusEffector> ConstPtr;

	ManeuverStatusEffector(const std::string &name) : Effector(name){};
	virtual ~ManeuverStatusEffector(){};

	virtual void setManeuverId(const int &id)
	{
		_maneuverId = id;
	}

	virtual void setStatus(const int &status)
	{
		_status = status;
	}

	virtual int getStatus() const
	{
		return _status;
	}

	virtual int getManeuverId() const
	{
		return _maneuverId;
	}

  protected:
	int _maneuverId = 0;
	int _status = -2;
};
}
