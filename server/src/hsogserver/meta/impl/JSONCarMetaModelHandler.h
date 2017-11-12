#pragma once

#include "../IActuatorConfig.h"
#include "../IAxleConfig.h"
#include "../ICameraConfig.h"
#include "../IDistanceSensorConfig.h"
#include "../ISensorConfig.h"
#include "../IServoDriveConfig.h"
#include "ConfigFactories/DistanceSensorConfigFactory.h"
#include "ConfigFactories/IConfigFactory.h"
#include "DistanceSensorConfig.h"
#include "JSONCarMetaModel.h"
#include <fstream>
#include <meta/impl/ConfigFactories/ActuatorConfigFactory.h>
#include <meta/impl/ConfigFactories/AxleConfigFactory.h>
#include <meta/impl/ConfigFactories/CameraConfigFactory.h>
#include <meta/impl/ConfigFactories/ServoDriveConfigFactory.h>
#include <rapidjson/rapidjson.h>
#include <rapidjson/reader.h>
#include <rapidjson/stream.h>
#include <string>

using namespace rapidjson;
using namespace std;
namespace taco
{
struct JSONCarMetaModelHandler
{
	string path;

	double floorHeight;
	std::string steeringServoName;
	std::string mainMotorName;
	IAxleConfig::ConstPtr frontAxle;
	IAxleConfig::ConstPtr rearAxle;

	std::vector<ISensorConfig::ConstPtr> gyroConfigs;
	std::vector<ISensorConfig::ConstPtr> accelerometerConfigs;
	std::vector<ISensorConfig::ConstPtr> imuConfigs;
	std::vector<IDistanceSensorConfig::ConstPtr> infraRedConfigs;
	std::vector<IDistanceSensorConfig::ConstPtr> ultrasonicConfigs;
	std::vector<ISensorConfig::ConstPtr> rotationConfigs;
	std::vector<ISensorConfig::ConstPtr> voltageConfigs;
	std::vector<ICameraConfig::ConstPtr> cameraConfigs;
	std::vector<ICameraConfig::ConstPtr> depthCameraConfigs;

	std::vector<IServoDriveConfig::ConstPtr> servoDriveConfigs;
	std::vector<IActuatorConfig::ConstPtr> motorConfigs;
	std::vector<IActuatorConfig::ConstPtr> lightConfigs;
	std::vector<IActuatorConfig::ConstPtr> statusConfigs;

	JSONCarMetaModelHandler(string filepath)
	{
		path = filepath;
		carmetafac = boost::make_shared<CarMetaModelFactory>();
		currentfac = boost::dynamic_pointer_cast<IConfigFactory>(carmetafac);
	}

	virtual ~JSONCarMetaModelHandler()
	{
	}

	bool Parse()
	{
		Reader r;
		ostringstream sstream;
		ifstream fs(path);
		sstream << fs.rdbuf();
		const std::string str(sstream.str());
		const char *ptr = str.c_str();
		StringStream s(ptr);
		r.Parse(s, *this);
		if (r.GetParseErrorCode() == ParseErrorCode::kParseErrorTermination) {
			return false;
		}
		return true;
	}

	bool Null()
	{
		return false;
	}
	bool Bool(bool b)
	{
		return false;
	}
	bool Int(int i)
	{
		bool ret = currentfac->Int(keys, i);
		keys.pop_back();
		return ret;
	}
	bool Uint(unsigned i)
	{
		bool ret = currentfac->Int(keys, (int) i);
		keys.pop_back();
		return ret;
	}
	bool Int64(int64_t i)
	{
		return false;
	}
	bool Uint64(uint64_t i)
	{
		return false;
	}
	bool Double(double d)
	{
		bool ret = currentfac->Double(keys, d);
		keys.pop_back();
		return ret;
	}
	bool RawNumber(const char *str, SizeType length, bool copy)
	{
		return false;
	}
	bool String(const char *str, SizeType length, bool copy)
	{
		bool ret = currentfac->String(keys, str);
		keys.pop_back();
		return ret;
	}
	bool StartObject()
	{
		if (keys.size() > 0) {
			if (keys.back().compare("ultrasonicConfigs") == 0) {
				currentfac = boost::make_shared<DistanceSensorConfigFactory>();
			} else if (keys.back().compare("lightConfigs") == 0 || keys.back().compare("motorConfigs") == 0 ||
					   keys.back().compare("statusConfigs") == 0) {
				currentfac = boost::make_shared<ActuatorConfigFactory>();
			} else if (keys.back().compare("rotationConfigs") == 0 || keys.back().compare("voltageConfigs") == 0 ||
					   keys.back().compare("imuConfigs") == 0) {
				currentfac = boost::make_shared<SensorConfigFactory>();
			} else if (keys.back().compare("cameraConfigs") == 0 || keys.back().compare("depthCameraConfigs") == 0) {
				currentfac = boost::make_shared<CameraConfigFactory>();
			} else if (keys.back().compare("frontAxle") == 0 || keys.back().compare("rearAxle") == 0) {
				currentfac = boost::make_shared<AxleConfigFactory>();
			} else if (keys.back().compare("servoDriveConfigs") == 0) {
				currentfac = boost::make_shared<ServoDriveConfigFactory>();
			}
		}
		return true;
	}
	bool Key(const char *str, SizeType length, bool copy)
	{
		keys.push_back(std::string(str, length));
		return true;
	}
	bool EndObject(SizeType memberCount)
	{
		if (keys.size() > 0) {
			const char *type = keys.back().c_str();
			if (std::string("ultrasonicConfigs").compare(type) == 0) {
				ultrasonicConfigs.push_back(boost::dynamic_pointer_cast<const IDistanceSensorConfig>(
						boost::dynamic_pointer_cast<ConfigFactory<taco::ISensorConfig::ConstPtr>>(currentfac)
								->getConfig()));
				return true;
			} else if (std::string("rotationConfigs").compare(type) == 0) {
				rotationConfigs.push_back(
						boost::dynamic_pointer_cast<ConfigFactory<taco::ISensorConfig::ConstPtr>>(currentfac)
								->getConfig());
				return true;
			} else if (std::string("voltageConfigs").compare(type) == 0) {
				voltageConfigs.push_back(
						boost::dynamic_pointer_cast<ConfigFactory<taco::ISensorConfig::ConstPtr>>(currentfac)
								->getConfig());
				return true;
			} else if (std::string("cameraConfigs").compare(type) == 0) {
				cameraConfigs.push_back(boost::dynamic_pointer_cast<const ICameraConfig>(
						boost::dynamic_pointer_cast<ConfigFactory<taco::ISensorConfig::ConstPtr>>(currentfac)
								->getConfig()));
				return true;
			} else if (std::string("depthCameraConfigs").compare(type) == 0) {
				depthCameraConfigs.push_back(boost::dynamic_pointer_cast<const ICameraConfig>(
						boost::dynamic_pointer_cast<ConfigFactory<taco::ISensorConfig::ConstPtr>>(currentfac)
								->getConfig()));
				return true;
			} else if (std::string("imuConfigs").compare(type) == 0) {
				imuConfigs.push_back(
						boost::dynamic_pointer_cast<ConfigFactory<taco::ISensorConfig::ConstPtr>>(currentfac)
								->getConfig());
				return true;
			} else if (std::string("frontAxle").compare(type) == 0) {
				frontAxle = boost::dynamic_pointer_cast<ConfigFactory<taco::IAxleConfig::ConstPtr>>(currentfac)
									->getConfig();
				keys.pop_back();
				return true;
			} else if (std::string("rearAxle").compare(type) == 0) {
				rearAxle = boost::dynamic_pointer_cast<ConfigFactory<taco::IAxleConfig::ConstPtr>>(currentfac)
								   ->getConfig();
				keys.pop_back();
				return true;
			} else if (std::string("lightConfigs").compare(type) == 0) {
				lightConfigs.push_back(
						boost::dynamic_pointer_cast<ConfigFactory<taco::IActuatorConfig::ConstPtr>>(currentfac)
								->getConfig());
				return true;
			} else if (std::string("motorConfigs").compare(type) == 0) {
				motorConfigs.push_back(
						boost::dynamic_pointer_cast<ConfigFactory<taco::IActuatorConfig::ConstPtr>>(currentfac)
								->getConfig());
				return true;
			} else if (std::string("statusConfigs").compare(type) == 0) {
				statusConfigs.push_back(
						boost::dynamic_pointer_cast<ConfigFactory<taco::IActuatorConfig::ConstPtr>>(currentfac)
								->getConfig());
				return true;
			} else if (std::string("servoDriveConfigs").compare(type) == 0) {
				servoDriveConfigs.push_back(boost::dynamic_pointer_cast<const IServoDriveConfig>(
						boost::dynamic_pointer_cast<ConfigFactory<taco::IActuatorConfig::ConstPtr>>(currentfac)
								->getConfig()));
				return true;
			} else {
				keys.pop_back();
			}
		} else {
			floorHeight = carmetafac->floorHeight;
			steeringServoName = carmetafac->steeringServoName;
			mainMotorName = carmetafac->mainMotorName;
		}
		return true;
	}
	bool StartArray()
	{
		return true;
	}
	bool EndArray(SizeType elementCount)
	{
		keys.pop_back();
		return true;
	}

  private:
	std::vector<std::string> keys;
	boost::shared_ptr<IConfigFactory> currentfac;
	boost::shared_ptr<CarMetaModelFactory> carmetafac;
};
}
