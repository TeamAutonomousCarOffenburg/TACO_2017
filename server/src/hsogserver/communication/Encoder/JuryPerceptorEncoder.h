#pragma once

#include "PerceptorEncoder.h"

// Codiert JuryPerceptors
class JuryPerceptorEncoder : public PerceptorEncoder<taco::IJuryPerceptor>
{
  public:
	JuryPerceptorEncoder(taco::IPerception::Ptr p, std::string perceptorName) : PerceptorEncoder(p, perceptorName)
	{
	}

	virtual ~JuryPerceptorEncoder()
	{
	}

  private:
	void writeValue(Writer<StringBuffer> *writer, taco::IJuryPerceptor::ConstPtr perceptor)
	{
		writer->Key("maneuverId");
		writer->Int(perceptor->getManeuverId());
		writer->Key("action");
		writer->Int(int(perceptor->getAction()));
	}
};
