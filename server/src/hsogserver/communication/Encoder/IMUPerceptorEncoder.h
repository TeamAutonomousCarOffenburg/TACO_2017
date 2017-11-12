#pragma once

#include "PerceptorEncoder.h"

class IMUPerceptorEncoder : public PerceptorEncoder<taco::IIMUPerceptor>
{
  public:
	IMUPerceptorEncoder(taco::IPerception::Ptr p, std::string perceptorName) : PerceptorEncoder(p, perceptorName)
	{
	}

	virtual ~IMUPerceptorEncoder(){};

  private:
	void writeValue(Writer<StringBuffer> *writer, taco::IIMUPerceptor::ConstPtr perceptor)
	{
		writer->Key("gyro");
		writer->StartObject();
		writer->Key("q0");
		writer->Double(perceptor->getGyro().w());
		writer->Key("q1");
		writer->Double(perceptor->getGyro().x());
		writer->Key("q2");
		writer->Double(perceptor->getGyro().y());
		writer->Key("q3");
		writer->Double(perceptor->getGyro().z());
		writer->EndObject();

		writer->Key("acceleration");
		writer->StartObject();
		writer->Key("x");
		writer->Double(perceptor->getAcceleration().x());
		writer->Key("y");
		writer->Double(perceptor->getAcceleration().y());
		writer->Key("z");
		writer->Double(perceptor->getAcceleration().z());
		writer->EndObject();
	}
};
