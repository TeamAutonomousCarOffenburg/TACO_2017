#pragma once

#include "EffectorHandler.h"

class PositionEffectorHandler : public EffectorHandler<taco::IPositionEffector>
{
  public:
	PositionEffectorHandler(taco::IAction::Ptr a, const std::string effectorName) : EffectorHandler(a, effectorName)
	{
	}

	virtual ~PositionEffectorHandler(){};

	bool handle(std::string name, void *value)
	{
		if (name.compare("posX") == 0) {
			effector->setPosX(*(double *) value);
			return true;
		} else if (name.compare("posY") == 0) {
			effector->setPosY(*(double *) value);
			return true;
		} else if (name.compare("angle") == 0) {
			effector->setAngle(*(double *) value);
			return true;
		}
		return false;
	}
};