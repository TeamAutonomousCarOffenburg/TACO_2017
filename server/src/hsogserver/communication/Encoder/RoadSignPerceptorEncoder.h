#pragma once

#include "EncoderUtilities.h"
#include "PerceptorEncoder.h"
#include <detection/signdetection/SignDetection.h>
#include <perception/IRoadSignPerceptor.h>
#include <utils/geometry/Pose2D.h>

using namespace taco;

class RoadSignPerceptorEncoder : public PerceptorEncoder<taco::IRoadSignPerceptor>
{
  public:
	RoadSignPerceptorEncoder(taco::IPerception::Ptr p, std::string perceptorName) : PerceptorEncoder(p, perceptorName)
	{
	}

	virtual ~RoadSignPerceptorEncoder()
	{
	}

  private:
	void writeValue(Writer<StringBuffer> *writer, taco::IRoadSignPerceptor::ConstPtr perceptor)
	{
		RoadSign detectedRoadSign = SignDetection::buildRoadSign(perceptor);
		// fix for receiving signs without valid angle
		if (std::isnan(detectedRoadSign.getPose().getAngle().rad())) {
			return;
		}
		writer->Key("signs");
		writer->StartArray();
		writer->StartObject();
		writer->Key("sign");
		writer->Int(static_cast<int>(detectedRoadSign.getSignType()));
		writer->Key("pose");
		EncoderUtilities::encodePose2D(writer, detectedRoadSign.getPose());
		writer->EndObject();
		writer->EndArray();
	}
};
