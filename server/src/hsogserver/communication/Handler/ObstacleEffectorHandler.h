#pragma once

#include "EffectorHandler.h"
#include <action/IObstacleEffector.h>

class ObstacleEffectorHandler : public EffectorHandler<taco::IObstacleEffector>
{
  public:
	ObstacleEffectorHandler(taco::IAction::Ptr a, const std::string effectorName) : EffectorHandler(a, effectorName)
	{
	}

	virtual ~ObstacleEffectorHandler(){};

	bool handle(std::string name, void *value)
	{
		if (name.compare("posX") == 0) {
			effector->setPosX(*(double *) value);
			return true;
		} else if (name.compare("posY") == 0) {
			effector->setPosY(*(double *) value);
			return true;
		}
		return false;
	}
};
