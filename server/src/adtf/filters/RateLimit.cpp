#include "RateLimit.h"
#include <iostream>

using namespace adtf;

ADTF_FILTER_PLUGIN("TACO Rate Limit", OID_ADTF_TACO_RATE, RateLimit);

RateLimit::~RateLimit()
{
}

RateLimit::RateLimit(const tChar *__info) : cFilter(__info)
{
	taco::IComponentFactory::ConstPtr factory(new taco::ComponentFactory());

	taco::ICarMetaModel::Ptr carMetaModel = factory->createCarMetaModel();
	taco::IAction::Ptr action = factory->createAction(carMetaModel);

	taco::ADTFPinMessageEncoder::Ptr encoder =
			taco::ADTFPinMessageEncoder::Ptr(new taco::ADTFPinMessageEncoder(action, carMetaModel));

	_carOutputPins = encoder->getOutputPins();
}

tResult RateLimit::CreatePins(__exception)
{
	int index = 0;
	for (auto it = _carOutputPins.begin(); it != _carOutputPins.end(); ++it, index++) {
		string inputName = (*it).name + "_input";
		string outputName = (*it).name + "_output";
		string sensorSignalType = (*it).signalType;

		cOutputPin *outputPin = new cOutputPin();
		RETURN_IF_FAILED(outputPin->Create(outputName.c_str(), new cMediaType(0, 0, 0, sensorSignalType.c_str()),
				static_cast<IPinEventSink *>(this)));
		RETURN_IF_FAILED(RegisterPin(outputPin));

		cInputPin *inputPin = new cInputPin();
		RETURN_IF_FAILED(inputPin->Create(inputName.c_str(), new cMediaType(0, 0, 0, sensorSignalType.c_str()),
				static_cast<IPinEventSink *>(this)));

		RETURN_IF_FAILED(RegisterPin(inputPin));
		_inputPinMap.insert(make_pair(inputPin, *it));
		_lastSentTime.insert(make_pair(inputPin, std::chrono::system_clock::now()));

		// save the pair of input / output so we can forward to the correct pin
		_inputPinToOutputMap.insert(make_pair(inputPin, outputPin));
	}
	RETURN_NOERROR;
}

tResult RateLimit::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample)
{
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		taco::OutputPin pin = _inputPinMap.at(pSource);
		string signalType = pin.signalType;

		IMediaTypeDescription *mediaTypeDescription = _descriptors[pSource];
		__adtf_sample_read_lock_mediadescription(mediaTypeDescription, pMediaSample, mediaCoder);

		bool changed_enough = false;
		if (signalType == "tSignalValue") {
			tFloat32 value = 0;
			mediaCoder->Get("f32Value", (tVoid *) &value);

			auto found = _tsignalLastValue.find(pSource);
			if (found != _tsignalLastValue.end()) {
				tFloat32 oldvalue = _tsignalLastValue[pSource];
				if (fabs(oldvalue - value) > 0.02) {
					changed_enough = true;
					_tsignalLastValue[pSource] = value;
				}
			} else {
				changed_enough = true;
				_tsignalLastValue[pSource] = value;
			}
		} else if (signalType == "tBoolSignalValue") {
			tBool value = mediaCoder->Get("tBoolSignalValue", (tVoid *) &value);

			auto found = _boolLastValue.find(pSource);
			if (found != _boolLastValue.end()) {
				tBool oldValue = _boolLastValue[pSource];
				if (value != oldValue) {
					changed_enough = true;
					_boolLastValue[pSource] = value;
				}
			} else {
				changed_enough = true;
				_boolLastValue[pSource] = value;
			}
		} else if (signalType == "tDriverStruct") {
			tInt8 stateId;
			mediaCoder->Get("i8StateID", (tVoid *) &stateId);
			tInt16 maneuverId;
			mediaCoder->Get("i16ManeuverEntry", (tVoid *) &maneuverId);
			auto found = _driverStructLastValue.find(pSource);
			if (found != _driverStructLastValue.end()) {
				std::pair<tInt8, tInt16> lastValue = _driverStructLastValue[pSource];
				if (lastValue.first != stateId || lastValue.second != maneuverId) {
					changed_enough = true;
					_driverStructLastValue[pSource] = std::make_pair(stateId, maneuverId);
				}

			} else {
				changed_enough = true;
				_driverStructLastValue[pSource] = std::make_pair(stateId, maneuverId);
			}
		}

		if (changed_enough || longEnoughSinceLastSent(pSource)) {
			_lastSentTime[pSource] = std::chrono::system_clock::now();
			cOutputPin *outputPin = _inputPinToOutputMap[pSource];
			outputPin->Transmit(pMediaSample);
		}
	}
	RETURN_NOERROR;
}

bool RateLimit::longEnoughSinceLastSent(IPin *pSource)
{
	std::chrono::system_clock::time_point last = _lastSentTime[pSource];
	std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

	long msSinceLast = std::chrono::duration_cast<std::chrono::milliseconds>(now - last).count();

	return msSinceLast > 1000;
}

tResult RateLimit::CreateDescriptors()
{
	_runtime->GetObject(
			OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid **) &_pDescManager, NULL);

	for (auto it = _inputPinMap.begin(); it != _inputPinMap.end(); ++it) {
		taco::OutputPin pin = (*it).second;
		tChar const *strDescSignalValue = _pDescManager->GetMediaDescription(pin.signalType.c_str());
		IMediaType *pTypeSignalValue = new cMediaType(
				0, 0, 0, pin.signalType.c_str(), strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
		pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &m_pCoderDescSignal);

		_descriptors[(*it).first] = m_pCoderDescSignal;
	}
	RETURN_NOERROR;
}

tResult RateLimit::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst) {
		RETURN_IF_FAILED(CreatePins(__exception_ptr));
	}
	if (eStage == StageGraphReady) {
		CreateDescriptors();
	}

	if (eStage == StageNormal) {
	}
	RETURN_NOERROR;
}

tResult RateLimit::Start(__exception)
{
	return cFilter::Start(__exception_ptr);
}

tResult RateLimit::Stop(__exception)
{
	return cFilter::Stop(__exception_ptr);
}

tResult RateLimit::Shutdown(tInitStage eStage, __exception)
{
	return cFilter::Shutdown(eStage, __exception_ptr);
}
