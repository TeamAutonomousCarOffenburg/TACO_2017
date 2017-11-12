#pragma once

#include <string>

namespace taco
{
struct InputPin
{
	std::string name;
	std::string signalType;
	bool received;

	InputPin(){};
	InputPin(std::string _name, std::string _signalType) : name(_name), signalType(_signalType), received(false){};

	bool operator==(const InputPin &other) const
	{
		return name == other.name && signalType == other.signalType;
	}
};
}
