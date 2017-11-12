#include "JSONCarMetaModel.h"
#include "JSONCarMetaModelHandler.h"

using namespace std;
using namespace taco;

JSONCarMetaModel::~JSONCarMetaModel()
{
}

JSONCarMetaModel::JSONCarMetaModel(double floorHeight, std::string steeringServoName, std::string mainMotorName,
		IAxleConfig::ConstPtr frontAxle, IAxleConfig::ConstPtr rearAxle,
		std::vector<ISensorConfig::ConstPtr> gyroConfigs, std::vector<ISensorConfig::ConstPtr> accelerometerConfigs,
		std::vector<ISensorConfig::ConstPtr> imuConfigs, std::vector<IDistanceSensorConfig::ConstPtr> infraRedConfigs,
		std::vector<IDistanceSensorConfig::ConstPtr> ultrasonicConfigs,
		std::vector<ISensorConfig::ConstPtr> rotationConfigs, std::vector<ISensorConfig::ConstPtr> voltageConfigs,
		std::vector<ICameraConfig::ConstPtr> cameraConfigs, std::vector<ICameraConfig::ConstPtr> depthCameraConfigs,
		std::vector<IServoDriveConfig::ConstPtr> servoDriveConfigs, std::vector<IActuatorConfig::ConstPtr> motorConfigs,
		std::vector<IActuatorConfig::ConstPtr> lightConfigs, std::vector<IActuatorConfig::ConstPtr> statusConfigs)
	: CarMetaModel(floorHeight, steeringServoName, mainMotorName)
{
	_frontAxle = frontAxle;
	_rearAxle = rearAxle;
	_gyroConfigs = gyroConfigs;
	_accelerometerConfigs = accelerometerConfigs;
	_imuConfigs = imuConfigs;
	_infraRedConfigs = infraRedConfigs;
	_ultrasonicConfigs = ultrasonicConfigs;
	_rotationConfigs = rotationConfigs;
	_voltageConfigs = voltageConfigs;
	_cameraConfigs = cameraConfigs;
	_depthCameraConfigs = depthCameraConfigs;
	_servoDriveConfigs = servoDriveConfigs;
	_motorConfigs = motorConfigs;
	_lightConfigs = lightConfigs;
	_statusConfigs = statusConfigs;
}
