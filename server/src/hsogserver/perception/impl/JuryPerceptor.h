#pragma once

#include <string>

#include "Perceptor.h"
#include "perception/IJuryPerceptor.h"

namespace taco
{
class JuryPerceptor : public Perceptor, public virtual IJuryPerceptor
{
  public:
	typedef boost::shared_ptr<JuryPerceptor> Ptr;
	typedef boost::shared_ptr<const JuryPerceptor> ConstPtr;

	JuryPerceptor(const std::string &name, const long &time, const int &action, const int &maneuverId)
		: Perceptor(name, time), _maneuver(maneuverId)
	{
		_action = static_cast<JuryAction>(action);
	};
	virtual ~JuryPerceptor(){};

	virtual const int getManeuverId() const
	{
		return _maneuver;
	}

	virtual const JuryAction getAction() const
	{
		return _action;
	}

  protected:
	int _maneuver = 0;
	JuryAction _action = JuryAction::GET_READY;
};
}
