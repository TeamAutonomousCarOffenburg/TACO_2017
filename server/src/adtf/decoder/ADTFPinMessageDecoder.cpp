#include "ADTFPinMessageDecoder.h"

#include "AccelerometerDecoder.h"
#include "CameraDecoder.h"
#include "DepthImageDecoder.h"
#include "GyroDecoder.h"
#include "IMUDecoder.h"
#include "JuryDecoder.h"
#include "ManeuverListDecoder.h"
#include "RoadSignDecoder.h"
#include "ValueGroupDecoder.h"
#include "WheelTickDecoder.h"

#include <boost/iterator/iterator_concepts.hpp>

using namespace taco;

ADTFPinMessageDecoder::ADTFPinMessageDecoder(
		IPerception::Ptr perception, ICarMetaModel::ConstPtr carMetaModel, IEventLogger::ConstPtr logger)
	: _perception(perception)
{
	// Add Sensor perceptor decoders
	for (auto &sensorConfig : carMetaModel->getIMUConfigs()) {
		_decoder.emplace_back(boost::make_shared<IMUDecoder>(sensorConfig, logger));
	}

	for (auto &sensorConfig : carMetaModel->getGyroConfigs()) {
		_decoder.emplace_back(boost::make_shared<GyroDecoder>(sensorConfig, logger));
	}

	for (auto &sensorConfig : carMetaModel->getAccelerometerConfigs()) {
		_decoder.emplace_back(boost::make_shared<AccelerometerDecoder>(sensorConfig, logger));
	}

	std::vector<IDistanceSensorConfig::ConstPtr> dsConfigs = carMetaModel->getInfraRedConfigs();
	if (!dsConfigs.empty()) {
		_decoder.emplace_back(boost::make_shared<ValueGroupDecoder>(dsConfigs, logger, 0.01));
	}

#if TACO_CONFIG == TACO_2017
	double conversionFactor = 0.01;
#else
	double conversionFactor = 1.0;
#endif

	dsConfigs = carMetaModel->getUltrasonicConfigs();
	if (!dsConfigs.empty()) {
		_decoder.emplace_back(boost::make_shared<ValueGroupDecoder>(dsConfigs, logger, conversionFactor));
	}

	for (auto &sensorConfig : carMetaModel->getRotationConfigs()) {
		_decoder.emplace_back(boost::make_shared<WheelTickDecoder>(sensorConfig, logger));
	}

	std::vector<ISensorConfig::ConstPtr> voltageConfigs = carMetaModel->getVoltageConfigs();
	if (!voltageConfigs.empty()) {
		_decoder.emplace_back(boost::make_shared<ValueGroupDecoder>(voltageConfigs, logger));
	}

	for (auto &depthCameraConfig : carMetaModel->getDepthCameraConfigs()) {
		_decoder.emplace_back(boost::make_shared<DepthImageDecoder>(depthCameraConfig, logger));
	}

	for (auto &cameraConfig : carMetaModel->getCameraConfigs()) {
		_decoder.emplace_back(boost::make_shared<CameraDecoder>(cameraConfig, logger));
	}

	_decoder.emplace_back(boost::make_shared<JuryDecoder>(logger));
	_decoder.emplace_back(boost::make_shared<RoadSignDecoder>(logger));
	_decoder.emplace_back(boost::make_shared<ManeuverListDecoder>(logger));

	// Add Actuator perceptor decoders
	std::vector<IServoDriveConfig::ConstPtr> servoDriveConfigs = carMetaModel->getServoDriveConfigs();
	if (!servoDriveConfigs.empty()) {
		_decoder.emplace_back(boost::make_shared<ValueGroupDecoder>(servoDriveConfigs, logger));
	}
}

ADTFPinMessageDecoder::~ADTFPinMessageDecoder() = default;

const vector<taco::InputPin> ADTFPinMessageDecoder::getPinConfigs()
{
	std::vector<InputPin> pinConfigs;

	for (const auto &it : _decoder) {
		const std::vector<InputPin> &newPinConfigs = it->getPinConfig();
		if (!newPinConfigs.empty()) {
			pinConfigs.insert(pinConfigs.begin(), newPinConfigs.begin(), newPinConfigs.end());
		}
	}

	return pinConfigs;
}

bool ADTFPinMessageDecoder::decode(
		const taco::InputPin &pin, adtf::IMediaTypeDescription *mediaTypeDescription, adtf::IMediaSample *mediaSample)
{
	for (const auto &it : _decoder) {
		if (it->indexOfPin(pin) >= 0) {
			std::vector<IPerceptor::ConstPtr> perceptors = it->decode(pin, mediaTypeDescription, mediaSample);

			if (!perceptors.empty()) {
				// Create perceptor map
				std::map<std::string, IPerceptor::ConstPtr> perceptorMap;
				for (auto &perceptor : perceptors) {
					perceptorMap.insert(make_pair(perceptor->getName(), perceptor));
				}

				// Publish new perceptions
				_perception->updatePerceptors(perceptorMap);
			}

			return true;
		}
	}

	return false;
}
