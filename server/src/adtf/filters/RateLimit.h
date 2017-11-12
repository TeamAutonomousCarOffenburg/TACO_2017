#pragma once

#include "../encoder/ADTFPinMessageEncoder.h"
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>

#include "general/ComponentFactory.h"
#include "general/IComponentFactory.h"

#include <chrono>

#define OID_ADTF_TACO_RATE "adtf.taco.RateLimit"

class RateLimit : public adtf::cFilter
{
	ADTF_DECLARE_FILTER_VERSION(
			OID_ADTF_TACO_RATE, "TACO Rate Limit", adtf::OBJCAT_DataFilter, "RateLimitVersion", 0, 0, 1, "1")

  public:
	RateLimit(const tChar *__info);
	virtual ~RateLimit();
	tResult Init(tInitStage eStage, __exception = NULL) override;
	tResult OnPinEvent(adtf::IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
			adtf::IMediaSample *pMediaSample) override;

  protected:
	tResult Start(__exception = NULL) override;
	tResult Stop(__exception = NULL) override;
	tResult Shutdown(tInitStage eStage, __exception = NULL) override;

  private:
	tResult CreatePins(__exception = NULL);
	tResult CreateDescriptors();

	bool longEnoughSinceLastSent(adtf::IPin *pSource);

	std::map<adtf::IPin *, taco::OutputPin> _inputPinMap;
	std::map<adtf::IPin *, adtf::cOutputPin *> _inputPinToOutputMap;
	std::map<adtf::IPin *, tFloat32> _tsignalLastValue;
	std::map<adtf::IPin *, tBool> _boolLastValue;
	std::map<adtf::IPin *, std::pair<tInt8, tInt16>> _driverStructLastValue;

	std::map<adtf::IPin *, std::chrono::system_clock::time_point> _lastSentTime;
	std::map<adtf::IPin *, cObjectPtr<adtf::IMediaTypeDescription>> _descriptors;

	std::vector<taco::OutputPin> _carOutputPins;
	adtf::IMediaDescriptionManager *_pDescManager;
};
