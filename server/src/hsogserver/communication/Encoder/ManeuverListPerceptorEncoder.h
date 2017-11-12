#pragma once

#include "PerceptorEncoder.h"
#include "perception/impl/Maneuver.h"

using namespace taco;

// Codiert ManeuverListPerceptors
class ManeuverListPerceptorEncoder : public PerceptorEncoder<taco::IManeuverListPerceptor>
{
  public:
	ManeuverListPerceptorEncoder(taco::IPerception::Ptr p, std::string perceptorName)
		: PerceptorEncoder(p, perceptorName)
	{
	}

	virtual ~ManeuverListPerceptorEncoder()
	{
	}

  private:
	void writeValue(Writer<StringBuffer> *writer, taco::IManeuverListPerceptor::ConstPtr perceptor)
	{
		writer->Key("maneuver");
		writer->StartArray();
		for (const taco::Maneuver maneuver : perceptor->getDriveInstructions()) {
			writer->StartObject();
			writer->Key("driveInstruction");
			writer->String(maneuver.getDriveInstruction().c_str());
			writer->Key("sector");
			writer->Int(maneuver.getSector());
			writer->Key("maneuverId");
			writer->Int(maneuver.getManeuverId());
			writer->EndObject();
		}
		writer->EndArray();
	}
};
