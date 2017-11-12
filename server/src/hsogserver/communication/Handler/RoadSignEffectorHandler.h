#pragma once

#include "EffectorHandler.h"
#include <action/IRoadSignEffector.h>

class RoadSignEffectorHandler : public EffectorHandler<taco::IRoadSignEffector>
{
  public:
	RoadSignEffectorHandler(taco::IAction::Ptr a, const std::string effectorName) : EffectorHandler(a, effectorName)
	{
	}

	virtual ~RoadSignEffectorHandler(){};

	bool handle(std::string name, void *value)
	{
		if (name.compare("id") == 0) {
			effector->setId(*(int *) value);
			return true;
		} else if (name.compare("posX") == 0) {
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