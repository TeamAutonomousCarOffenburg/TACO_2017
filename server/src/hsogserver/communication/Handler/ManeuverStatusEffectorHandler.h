#pragma once

#include "EffectorHandler.h"

class ManeuverStatusEffectorHandler : public EffectorHandler<taco::IManeuverStatusEffector>
{
  public:
	ManeuverStatusEffectorHandler(taco::IAction::Ptr a, const std::string effectorName)
		: EffectorHandler(a, effectorName)
	{
	}

	virtual ~ManeuverStatusEffectorHandler(){};

	bool handle(std::string name, void *value)
	{
		if (name.compare("maneuverId") == 0) {
			effector->setManeuverId(*(int *) value);
			return true;
		} else if (name.compare("status") == 0) {
			effector->setStatus(*(int *) value);
			return true;
		}
		return false;
	}
};
