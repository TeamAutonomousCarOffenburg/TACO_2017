#pragma once

#include "../../action/IAction.h"
#include "IHandler.h"

template <class EFF> class EffectorHandler : public IHandler
{
  public:
	std::string getName()
	{
		return effectorName;
	}

	virtual ~EffectorHandler(){};

  protected:
	EffectorHandler(taco::IAction::Ptr a, std::string effectorName)
	{
		effector = boost::dynamic_pointer_cast<EFF>(a->getEffector(effectorName));
		this->effectorName = effectorName;
	}
	typename EFF::Ptr effector;

  private:
	std::string effectorName;
};
