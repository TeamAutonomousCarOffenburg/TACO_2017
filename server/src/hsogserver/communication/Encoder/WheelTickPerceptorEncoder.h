#pragma once

#include "PerceptorEncoder.h"

// Codiert WheelTickPerceptors
class WheelTickPerceptorEncoder : public PerceptorEncoder<taco::IWheelTickPerceptor>
{
  public:
	WheelTickPerceptorEncoder(taco::IPerception::Ptr p, std::string perceptorName) : PerceptorEncoder(p, perceptorName)
	{
	}

	virtual ~WheelTickPerceptorEncoder(){};

  private:
	void writeValue(Writer<StringBuffer> *writer, taco::IWheelTickPerceptor::ConstPtr perceptor)
	{
		writer->Key("ticks");
		writer->Int(perceptor->getTicks());
		writer->Key("direction");
		writer->Int(perceptor->getDirection());
	}
};
