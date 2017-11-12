#pragma once

#include "action/IEffector.h"

namespace taco
{
class Effector : public virtual IEffector
{
  public:
	typedef boost::shared_ptr<Effector> Ptr;
	typedef boost::shared_ptr<const Effector> ConstPtr;

	Effector(const std::string &name) : _name(name), _time(0){};
	virtual ~Effector(){};

	virtual const std::string &getName() const
	{
		return _name;
	};

	virtual const long &getTime() const
	{
		return _time;
	};

  protected:
	/** The name of the effector. */
	std::string _name;

	/** The time of the last effector update. */
	long _time;
};
}
