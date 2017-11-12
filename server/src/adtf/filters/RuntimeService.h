#pragma once

#include <thread>

// Include order seems to matter here...
// clang-format off
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <adtf_graphics.h>
// clang-format on

#include <communication/Communication.h>

#include "../decoder/ADTFPinMessageDecoder.h"
#include "../encoder/ADTFPinMessageEncoder.h"

#include "detection/FloorNormalDetection.h"
#include "detection/ILaneDetection.h"
#include "detection/IObjectDetection.h"
#include "../../hsogserver/action/IValueEffector.h"

#define OID_TACO_RUNTIME_SERVICE "adtf.taco.RuntimeService"
#define PROPERTY_START_CLIENT "Start Client"
#define PROPERTY_START_VISION "Start Vision"
#define PROPERTY_CLIENT_LOGGING "Client Logging"
#define PROPERTY_CLIENT_SCENARIO "Client Scenario"

class RuntimeService : public cFilter
{
	ADTF_DECLARE_FILTER_VERSION(OID_TACO_RUNTIME_SERVICE, "TACO Runtime Service", OBJCAT_DataFilter,
			"tacoRuntimeServiceVersion", 0, 0, 1, "1")

  public:
	RuntimeService(const tChar *__info);
	virtual ~RuntimeService();

	tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample) override;

	tResult Init(tInitStage eStage, __exception = NULL) override;
	tResult Start(__exception = NULL) override;
	tResult Stop(__exception = NULL) override;
	tResult Shutdown(tInitStage eStage, __exception = NULL) override;

  private:
	taco::ADTFPinMessageDecoder::Ptr _decoder;
	taco::ADTFPinMessageEncoder::Ptr _encoder;

	taco::ICarMetaModel::Ptr _carMetaModel;
	taco::IAction::Ptr _action;
	taco::IPerception::Ptr _perception;
	taco::IEventLogger::ConstPtr _logger;
	taco::ILaneDetection::Ptr _laneDetection;
	taco::FloorNormalDetection _floorNormalDetection;
	taco::IObjectDetection::Ptr _objectDetection;

	tResult CreateInputPins(__exception = NULL);
	tResult CreateOutputPins(__exception = NULL);
	tResult CreateDescriptors();
	tResult SendToActuators();

	std::map<IPin *, taco::InputPin> _inputPinMap;
	std::map<IPin *, taco::OutputPin> _outputPinMap;
	std::map<IPin *, cObjectPtr<IMediaTypeDescription>> _descriptors;

	std::map<string, double> _usPerceptorValues;

	std::thread _actuatorThread;
	bool _running = false;
	bool _init = false;
	bool _clientStarted = false;
	bool _visionStarted = false;
	void actuatorThread();

	void CreateRuntime();
	void StartClient();
	void StartVision();
	void RunCommandInDirectory(string command, string directory);

	Communication *communication;
};
