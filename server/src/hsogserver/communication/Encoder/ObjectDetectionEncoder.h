#pragma once

#include "EncoderUtilities.h"
#include "IEncoder.h"
#include "detection/IObjectDetection.h"

class ObjectDetectionEncoder : public IEncoder
{
  private:
	taco::IObjectDetection::Ptr _objectDetection;

  public:
	ObjectDetectionEncoder(taco::IObjectDetection::Ptr objectDetection)
	{
		_objectDetection = objectDetection;
	}

	virtual ~ObjectDetectionEncoder(){};

	virtual void encode(Writer<StringBuffer> *writer)
	{
		auto result = _objectDetection->getObjectResult();
		if (result.empty()) {
			return;
		}
		writer->Key("Objects");
		writer->StartObject();
		writer->Key("objects");
		writer->StartArray();
		for (auto it = result.begin(); it != result.end(); it++) {
			writer->StartObject();
			writer->Key("name");
			writer->String(it->get()->getName().c_str());
			writer->Key("bounds");
			EncoderUtilities::encodePolygon(writer, it->get()->getBoundingPoly());
			writer->Key("pose");
			EncoderUtilities::encodePose2D(writer, it->get()->getPose());
			writer->EndObject();
		}
		writer->EndArray();
		writer->EndObject();
	}
};
