#pragma once

#include "EncoderUtilities.h"
#include "IEncoder.h"
#include "detection/FloorNormalDetection.h"
#include <AADCCar.h>
#include <perception/IPerception.h>

class FloorNormalEncoder : public IEncoder
{
  private:
	taco::IPerception::Ptr _per;
	taco::FloorNormalDetection &_floorNormal;

  public:
	FloorNormalEncoder(taco::IPerception::Ptr per, taco::FloorNormalDetection &floorNormal)
		: _per(per), _floorNormal(floorNormal)
	{
	}

	virtual ~FloorNormalEncoder(){};

	virtual void encode(Writer<StringBuffer> *writer)
	{
		auto pcp = _per->getPointCloudPerceptor(taco::AADCCar::XTION_CAMERA_DEPTH);
		if (pcp) {
			writer->Key("FloorNormal");
			writer->StartObject();
			writer->Key("value");
			// TODO: we should probably not re-calculate this...?
			EncoderUtilities::encodeVector3d(writer, _floorNormal.calculateFloorNormal(pcp));
			writer->EndObject();
		}
	}
};
