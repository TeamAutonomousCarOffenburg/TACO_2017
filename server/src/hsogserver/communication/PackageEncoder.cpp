#include "PackageEncoder.h"
#include "../AADCCar.h"
#include "ByteConverter.h"
#include "Encoder/DoubleValuePerceptorEncoder.h"
#include "Encoder/IMUPerceptorEncoder.h"
#include "Encoder/JuryPerceptorEncoder.h"
#include "Encoder/ManeuverListPerceptorEncoder.h"
#include "Encoder/RoadSignPerceptorEncoder.h"
#include "Encoder/WheelTickPerceptorEncoder.h"
#include "detection/ILaneDetection.h"
#include <communication/Encoder/EnvironmentConfigEncoder.h>
#include <communication/Encoder/FloorNormalEncoder.h>
#include <communication/Encoder/LaneAssistEncoder.h>
#include <communication/Encoder/ObjectDetectionEncoder.h>

using namespace rapidjson;
using namespace std::chrono;

size_t PackageEncoder::encode(unsigned char *buf, system_clock::time_point lastClientConnectionTime)
{
	StringBuffer s;
	Writer<StringBuffer> writer(s);

	writer.StartObject();

	// Time in seconds since client has connected
	writer.Key("time");
	writer.StartObject();
	writer.Key("time");
	system_clock::time_point currentTime = system_clock::now();
	duration<double> timeSinceClientConnected = currentTime - lastClientConnectionTime;
	writer.Double(timeSinceClientConnected.count());
	writer.EndObject();
	// Codiert alle Wert mithilfe der Encoderliste
	for (auto it = Encoderlist.begin(); it != Encoderlist.end(); it++) {
		it.operator*()->encode(&writer);
	}
	writer.EndObject();

	memcpy(buf + 4, s.GetString(), s.GetSize());
	ByteConverter::IntToBytes(static_cast<int>(s.GetSize()), buf);

	return s.GetSize() + 4;
}

PackageEncoder::PackageEncoder(taco::IPerception::Ptr p, taco::ICarMetaModel::Ptr cm,
		taco::ILaneDetection::Ptr laneDetection, taco::IObjectDetection::Ptr objectDetection,
		taco::FloorNormalDetection &floorNormal, taco::EnvironmentConfiguration::Ptr &environmentConfig)
{
	for (auto &config : cm->getUltrasonicConfigs()) {
		Encoderlist.push_back(new DoubleValuePerceptorEncoder(p, config->getPerceptorName()));
	}

	for (auto &config : cm->getVoltageConfigs()) {
		Encoderlist.push_back(new DoubleValuePerceptorEncoder(p, config->getPerceptorName()));
	}

	for (auto &config : cm->getServoDriveConfigs()) {
		Encoderlist.push_back(new DoubleValuePerceptorEncoder(p, config->getPerceptorName()));
	}

	for (auto &config : cm->getIMUConfigs()) {
		Encoderlist.push_back(new IMUPerceptorEncoder(p, config->getPerceptorName()));
	}

	for (auto &config : cm->getRotationConfigs()) {
		Encoderlist.push_back(new WheelTickPerceptorEncoder(p, config->getPerceptorName()));
	}

	for (auto &config : cm->getManeuverStatusConfigs()) {
		Encoderlist.push_back(new ManeuverListPerceptorEncoder(p, config->getPerceptorName()));
	}

	// TODO: aren't defined in ICarMetaModel... maybe have to be generated from somewhere else.
	Encoderlist.push_back(new ManeuverListPerceptorEncoder(p, taco::AADCCar::MANEUVER_LIST));
	Encoderlist.push_back(new JuryPerceptorEncoder(p, taco::AADCCar::JURY_COMMAND));
	Encoderlist.push_back(new RoadSignPerceptorEncoder(p, taco::AADCCar::SIGNS));

	Encoderlist.push_back(new LaneAssistEncoder(laneDetection));
	// Encoderlist.push_back(new ObjectDetectionEncoder(objectDetection));
	// Encoderlist.push_back(new FloorNormalEncoder(p, floorNormal));
	Encoderlist.push_back(new EnvironmentConfigEncoder(environmentConfig));
}

PackageEncoder::~PackageEncoder()
{
	for (auto it = Encoderlist.begin(); it != Encoderlist.end(); it++) {
		delete it.operator*();
	}
}