#pragma once

#include "EffectorHandler.h"
#include <chrono>

class BoolEffectorHandler : public EffectorHandler<taco::IBoolValueEffector>
{
  public:
	BoolEffectorHandler(taco::IAction::Ptr a, const std::string effectorName) : EffectorHandler(a, effectorName)
	{
	}

	virtual ~BoolEffectorHandler(){};

	bool handle(std::string name, void *value)
	{
		if (this->getName().compare(name) == 0) {
			bool boolValue = *(bool *) value;

			unsigned long milliseconds_since_epoch = static_cast<unsigned long>(
					std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1));

			effector->setValue(boolValue, milliseconds_since_epoch);
			return true;
		}
		return false;
	}
};
