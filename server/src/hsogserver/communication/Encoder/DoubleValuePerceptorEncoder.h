#pragma once

#include "PerceptorEncoder.h"

using namespace rapidjson;

// Codiert DoubleValuePerceptors
class DoubleValuePerceptorEncoder : public PerceptorEncoder<taco::IDoubleValuePerceptor>
{
  public:
	DoubleValuePerceptorEncoder(taco::IPerception::Ptr p, std::string perceptorName)
		: PerceptorEncoder(p, perceptorName)
	{
	}
	virtual ~DoubleValuePerceptorEncoder(){};

  private:
	void writeValue(Writer<StringBuffer> *writer, taco::IDoubleValuePerceptor::ConstPtr perceptor)
	{
		writer->Key("value");
		writer->Double(perceptor.get()->getValue());
	}
};
