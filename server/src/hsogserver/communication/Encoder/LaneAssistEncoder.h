#pragma once

#include "EncoderUtilities.h"
#include "IEncoder.h"
#include "detection/ILaneDetection.h"

class LaneAssistEncoder : public IEncoder
{
  private:
	taco::ILaneDetection::Ptr _detection;

	void encodeLaneMiddle(Writer<StringBuffer> *writer, taco::LaneMiddle middle)
	{
		writer->StartObject();
		writer->Key("valid");
		writer->Bool(middle.valid);
		writer->Key("confidence");
		writer->Double(middle.confidence);
		writer->Key("invalidSince");
		writer->Int(middle.invalidSince);
		writer->Key("wantedX");
		writer->Int(middle.wantedX);
		writer->Key("middleX");
		writer->Int(middle.middleX);
		writer->Key("middleXCamera");
		EncoderUtilities::encodeVector3d(writer, middle.middleXCamera);
		writer->Key("rightLineX");
		writer->Int(middle.rightLineX);
		writer->Key("middleLineX");
		writer->Int(middle.middleLineX);
		writer->Key("leftLineX");
		writer->Int(middle.leftLineX);
		writer->Key("scanRightStartX");
		writer->Int(middle.scanRightStartX);
		writer->Key("scanRightStartY");
		writer->Int(middle.scanRightStartY);
		writer->Key("scanRightEndX");
		writer->Int(middle.scanRightEndX);
		writer->Key("inCrossing");
		writer->Bool(middle.inCrossing);
		writer->EndObject();
	}

  public:
	LaneAssistEncoder(taco::ILaneDetection::Ptr detection)
	{
		_detection = detection;
	}

	virtual ~LaneAssistEncoder(){};

	virtual void encode(Writer<StringBuffer> *writer)
	{
		auto result = _detection->getLaneResult();
		if (!result) {
			return;
		}

		writer->Key("laneMiddle");
		encodeLaneMiddle(writer, result->laneMiddle);
	}
};
