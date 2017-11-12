#pragma once

#include "EffectorHandler.h"

class DoubleEffectorHandler : public EffectorHandler<taco::IDoubleValueEffector>
{
  public:
	DoubleEffectorHandler(taco::IAction::Ptr a, const std::string effectorName) : EffectorHandler(a, effectorName)
	{
	}

	virtual ~DoubleEffectorHandler(){};

	bool handle(std::string name, void *value)
	{
		if (this->getName().compare(name) == 0) {
			double doubleValue = *(double *) value;
			effector->setValue(doubleValue, 0);
			return true;
		}
		return false;
	}
};
