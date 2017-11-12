#include "RuntimeService.h"
#include "general/ComponentFactory.h"
#include "general/LogComponentFactory.h"
#include <AADCCar.h>
#include <sys/prctl.h>

ADTF_FILTER_PLUGIN("TACO Runtime Service", OID_TACO_RUNTIME_SERVICE, RuntimeService);

using namespace taco;
using namespace std;
using namespace std::chrono;

RuntimeService::RuntimeService(const tChar *__info) : cFilter(__info), _floorNormalDetection(30.0)
{
#ifdef __linux__
	prctl(PR_SET_NAME, "RuntimeService", 0, 0, 0);
#endif

	SetPropertyBool(PROPERTY_START_CLIENT, tTrue);
	SetPropertyBool(PROPERTY_START_VISION, tTrue);
	SetPropertyBool(PROPERTY_CLIENT_LOGGING, tTrue);
	SetPropertyStr(PROPERTY_CLIENT_SCENARIO, "");
}

void RuntimeService::CreateRuntime()
{
	// Check for unique initialization
	if (!_init) {
		// IComponentFactory::ConstPtr factory(new ComponentFactory());
		IComponentFactory::ConstPtr factory(new LogComponentFactory());
		_init = true;
		_carMetaModel = factory->createCarMetaModel();
		_action = factory->createAction(_carMetaModel);
		_perception = factory->createPerception(_carMetaModel);
		_laneDetection = factory->createLaneDetection(_carMetaModel);
		// _objectDetection = factory->createObjectDetection(_carMetaModel);
		_logger = factory->createEventLogger();
		// Init default perceptor values
		_usPerceptorValues.insert(make_pair(AADCCar::US_FRONT_CENTER_LEFT, -1));
		_usPerceptorValues.insert(make_pair(AADCCar::US_FRONT_CENTER, -1));
		_usPerceptorValues.insert(make_pair(AADCCar::US_FRONT_CENTER_RIGHT, -1));
	}

	_encoder = ADTFPinMessageEncoder::Ptr(new ADTFPinMessageEncoder(_action, _carMetaModel));
	_decoder = ADTFPinMessageDecoder::Ptr(new ADTFPinMessageDecoder(_perception, _carMetaModel, _logger));
}

RuntimeService::~RuntimeService() = default;

tResult RuntimeService::CreateInputPins(__exception)
{
	vector<InputPin> inputPins = _decoder->getPinConfigs();

	int index = 0;
	for (auto it = inputPins.begin(); it != inputPins.end(); ++it, index++) {
		string sensorName = (*it).name;
		string sensorSignalType = (*it).signalType;

		cPin *pin;
		if (sensorSignalType != "VIDEO") {
			pin = new cInputPin();
			RETURN_IF_FAILED(((cInputPin *) pin)
									 ->Create(sensorName.c_str(), new cMediaType(0, 0, 0, sensorSignalType.c_str()),
											 static_cast<IPinEventSink *>(this)));
		} else {
			pin = new cVideoPin();
			RETURN_IF_FAILED(
					((cVideoPin *) pin)
							->Create(sensorName.c_str(), adtf::IPin::PD_Input, static_cast<IPinEventSink *>(this)));
		}

		RETURN_IF_FAILED(RegisterPin(pin));
		_inputPinMap.insert(make_pair(pin, *it));
	}
	RETURN_NOERROR;
}

tResult RuntimeService::CreateDescriptors()
{
	IMediaDescriptionManager *descManager;
	_runtime->GetObject(
			OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid **) &descManager, nullptr);

	for (auto &inputPin : _inputPinMap) {
		InputPin pin = inputPin.second;
		tChar const *strDescSignalValue = descManager->GetMediaDescription(pin.signalType.c_str());
		IMediaType *pTypeSignalValue = new cMediaType(
				0, 0, 0, pin.signalType.c_str(), strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
		pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &m_pCoderDescSignal);

		_descriptors[inputPin.first] = m_pCoderDescSignal;
	}

	for (auto &outputPin : _outputPinMap) {
		OutputPin pin = outputPin.second;
		tChar const *strDescSignalValue = descManager->GetMediaDescription(pin.signalType.c_str());
		IMediaType *pTypeSignalValue = new cMediaType(
				0, 0, 0, pin.signalType.c_str(), strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
		pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &m_pCoderDescSignal);

		_descriptors[outputPin.first] = m_pCoderDescSignal;
	}

	RETURN_NOERROR;
}

tResult RuntimeService::CreateOutputPins(__exception)
{
	vector<OutputPin> outputPins = _encoder->getOutputPins();

	int index = 0;
	for (auto it = outputPins.begin(); it != outputPins.end(); ++it, index++) {
		string sensorName = (*it).name + "_output";
		string sensorSignalType = (*it).signalType;

		auto *pin = new cOutputPin();
		RETURN_IF_FAILED(pin->Create(sensorName.c_str(), new cMediaType(0, 0, 0, sensorSignalType.c_str()),
				static_cast<IPinEventSink *>(this)));
		RETURN_IF_FAILED(RegisterPin(pin));

		_outputPinMap.insert(make_pair(pin, *it));
	}
	RETURN_NOERROR;
}

tResult RuntimeService::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst) {
		CreateRuntime();

		RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
		RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));
	}

	if (eStage == StageGraphReady) {
		CreateDescriptors();
	}
	RETURN_NOERROR;
}

void RuntimeService::actuatorThread()
{
	system_clock::time_point startTime = system_clock::now();

	system_clock::time_point beforeCycle;
	system_clock::time_point afterCycle;

	// workaround for #44
	static bool floorNormalInitialized = true;
	static Eigen::Vector3d floorNormal = Eigen::Vector3d(0, 0, 1);

	while (_running) {
		long elapsedTimeMs;
		beforeCycle = system_clock::now();

		if (beforeCycle - startTime > seconds(5)) {
			if (!_clientStarted && GetPropertyBool(PROPERTY_START_CLIENT)) {
				StartClient();
				_clientStarted = true;
			}

			if (!_visionStarted && GetPropertyBool(PROPERTY_START_VISION)) {
				StartVision();
				_visionStarted = true;
			}
		}

		SendToActuators();

		while (_perception->nextPerceptorMap()) {
			auto pointCloudPerceptor = _perception->getPointCloudPerceptor(AADCCar::XTION_CAMERA_DEPTH);
			if (pointCloudPerceptor) {
				// floorNormal = _floorNormalDetection.calculateFloorNormal(pointCloudPerceptor);
				floorNormalInitialized = true;
			}
#if TACO_CONFIG == TACO_2017
			string camera = AADCCar::BASLER_CAMERA;
#else
			string camera = AADCCar::XTION_CAMERA_RGB;
#endif
			auto cameraPerceptor = _perception->getCameraPerceptor(camera);
			if (cameraPerceptor) {
				cv::Mat frame = cameraPerceptor->getImage();
				if (floorNormalInitialized) {
					_laneDetection->setData(frame, floorNormal);
					_laneDetection->notifyPossibleNewFrame();
				}

				vector<IDoubleValuePerceptor::ConstPtr> sensorsIR;
				vector<IDoubleValuePerceptor::ConstPtr> sensorsUS;

				for (const auto &ultrasonic : _carMetaModel->getUltrasonicConfigs()) {
					auto perceptor = _perception->getDoubleValuePerceptor(ultrasonic->getName());
					if (perceptor) {
						sensorsUS.push_back(perceptor);
					}
				}
				// _objectDetection->setData(
				//		_perception->getPointCloudPerceptor(AADCCar::XTION_CAMERA_DEPTH), sensorsIR, sensorsUS);
				// _objectDetection->update();
			}
			communication->update();
		}
		afterCycle = system_clock::now();

		elapsedTimeMs = duration_cast<milliseconds>(afterCycle - beforeCycle).count();

		milliseconds duration(20 - elapsedTimeMs);
		this_thread::sleep_for(duration);
	}
}

tResult RuntimeService::Start(__exception)
{
	_laneDetection->start();
	// _objectDetection->start();
	CreateRuntime();
	communication = new Communication(
			_carMetaModel, _action, _perception, _laneDetection, _objectDetection, _floorNormalDetection);
	communication->start();
	_running = true;
	_actuatorThread = thread(&RuntimeService::actuatorThread, this);

#ifdef __linux__
	auto handle = _actuatorThread.native_handle();
	pthread_setname_np(handle, "ActuatorThread");
#endif

	return cFilter::Start(__exception_ptr);
}

void RuntimeService::StartClient()
{
	string metaModelDirectory = string(TACO_DIRECTORY) + "/config";
#if TACO_CONFIG == TACO_2017
	string version = "taco2017";
#elif TACO_CONFIG == TACO_2016
	string version = "taco2016";
#endif

	string command = string("bash start.sh --metaModelDirectory=") + metaModelDirectory + " --version=" + version;

	cString scenario = GetPropertyStr(PROPERTY_CLIENT_SCENARIO);
	if (!scenario.IsEmpty()) {
		command = command + " --scenario=" + scenario.GetPtr();
	}

	if (GetPropertyBool(PROPERTY_CLIENT_LOGGING)) {
		command = command + " --log";
	}

	string directory = string(TACO_DIRECTORY) + "/client/jar/out/tacoAgent";

	RunCommandInDirectory(command, directory);
}

void RuntimeService::StartVision()
{
	RunCommandInDirectory(string("bash start.sh &"), string(TACO_DIRECTORY) + "/vision");
}

void RuntimeService::RunCommandInDirectory(string command, string directory)
{
	char cwd[1024];
	if (getcwd(cwd, sizeof(cwd)) == nullptr) {
		cerr << "RunCommandInDirectory(): unable to get the working directory: " << strerror(errno) << endl;
		return;
	}

	if (chdir(directory.c_str()) == -1) {
		cerr << "RunCommandInDirectory(): chdir(" << directory << ") failed: " << strerror(errno) << endl;
		return;
	}

	cout << directory << ": " << command << endl;
	if (system(command.c_str()) == -1) {
		cerr << "RunCommandInDirectory(): system() failed: " << strerror(errno) << endl;
	}

	if (chdir(cwd) == -1) {
		cerr << "RunCommandInDirectory(): chdir(" << cwd << ") failed: " << strerror(errno) << endl;
		return;
	}
}

tResult RuntimeService::Stop(__exception)
{
	try {
		_running = false;
		_laneDetection->stop();
		//_objectDetection->stop();
		_actuatorThread.join();
		communication->stop();
		delete communication;

		_decoder.reset();
		_encoder.reset();
	} catch (const exception &ex) {
		cout << " exception: " << ex.what() << endl;
	} catch (...) {
		exception_ptr eptr = current_exception();

		try {
			rethrow_exception(eptr);
		} catch (const exception &e) {
			cout << " exception: " << e.what() << endl;
		}
	}

	return cFilter::Stop(__exception_ptr);
}

tResult RuntimeService::Shutdown(tInitStage eStage, __exception)
{
	if (eStage == StageGraphReady) {
	}
	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult RuntimeService::OnPinEvent(
		IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample)
{
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived && _running) {
		auto pin = _inputPinMap.find(pSource);
		if (pin != _inputPinMap.end()) {
			IMediaTypeDescription *mediaTypeDescription = _descriptors[pSource];
			_decoder->decode(pin->second, mediaTypeDescription, pMediaSample);
		} else {
			cerr << "could not find matching pin " << endl;
		}
	}
	RETURN_NOERROR;
}

tResult RuntimeService::SendToActuators()
{
	list<pair<IPin *, OutputPin>> pinsByModificationTime;
	for (auto &pin : _outputPinMap) {
		pinsByModificationTime.emplace_back(pin.first, pin.second);
	}

	// sort by modification time so that most recently modified effectors have priority (#27)
	pinsByModificationTime.sort([&](const pair<IPin *, OutputPin> &a, const pair<IPin *, OutputPin> &b) {
		return _action->getEffector(a.second.name)->getTime() < _action->getEffector(b.second.name)->getTime();
	});

	for (auto &pin : pinsByModificationTime) {
		IMediaTypeDescription *descriptor = _descriptors[pin.first];

		cObjectPtr<IMediaSample> mediaSample;
		AllocMediaSample(&mediaSample);

		if (_encoder->encode(pin.second, descriptor, mediaSample)) {
			pin.first->Transmit(mediaSample);
		}
	}
	RETURN_NOERROR;
}
