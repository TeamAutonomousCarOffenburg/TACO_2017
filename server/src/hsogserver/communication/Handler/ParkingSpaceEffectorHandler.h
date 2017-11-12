#pragma once

#include "EffectorHandler.h"
#include <action/IParkingSpaceEffector.h>

class ParkingSpaceEffectorHandler : public EffectorHandler<taco::IParkingSpaceEffector>
{
  public:
	ParkingSpaceEffectorHandler(taco::IAction::Ptr a, const std::string effectorName) : EffectorHandler(a, effectorName)
	{
	}

	virtual ~ParkingSpaceEffectorHandler(){};

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
		} else if (name.compare("state") == 0) {
			effector->setState(*(int *) value);
			return true;
		}
		return false;
	}
};