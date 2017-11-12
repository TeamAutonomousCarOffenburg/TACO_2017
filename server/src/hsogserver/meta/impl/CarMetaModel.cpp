#include "CarMetaModel.h"

using namespace taco;

CarMetaModel::CarMetaModel(
		const double &floorHeight, const std::string &steeringServoName, const std::string &mainMotorName)
	: _floorHeight(floorHeight), _steeringServoName(steeringServoName), _mainMotorName(mainMotorName)
{
}

CarMetaModel::~CarMetaModel()
{
}

const std::string &CarMetaModel::getSteeringServoName() const
{
	return _steeringServoName;
}

const std::string &CarMetaModel::getMainMotorName() const
{
	return _mainMotorName;
}

const double &CarMetaModel::getFloorHeight() const
{
	return _floorHeight;
}

IAxleConfig::ConstPtr CarMetaModel::getFrontAxle() const
{
	return _frontAxle;
}

IAxleConfig::ConstPtr CarMetaModel::getRearAxle() const
{
	return _rearAxle;
}

const std::vector<ISensorConfig::ConstPtr> &CarMetaModel::getGyroConfigs() const
{
	return _gyroConfigs;
}

const std::vector<ISensorConfig::ConstPtr> &CarMetaModel::getAccelerometerConfigs() const
{
	return _accelerometerConfigs;
}

const std::vector<ISensorConfig::ConstPtr> &CarMetaModel::getIMUConfigs() const
{
	return _imuConfigs;
}

const std::vector<IDistanceSensorConfig::ConstPtr> &CarMetaModel::getInfraRedConfigs() const
{
	return _infraRedConfigs;
}

const std::vector<IDistanceSensorConfig::ConstPtr> &CarMetaModel::getUltrasonicConfigs() const
{
	return _ultrasonicConfigs;
}

const std::vector<ISensorConfig::ConstPtr> &CarMetaModel::getRotationConfigs() const
{
	return _rotationConfigs;
}

const std::vector<ISensorConfig::ConstPtr> &CarMetaModel::getVoltageConfigs() const
{
	return _voltageConfigs;
}

const std::vector<ICameraConfig::ConstPtr> &CarMetaModel::getCameraConfigs() const
{
	return _cameraConfigs;
}

const std::vector<ICameraConfig::ConstPtr> &CarMetaModel::getDepthCameraConfigs() const
{
	return _depthCameraConfigs;
}

const std::vector<IServoDriveConfig::ConstPtr> &CarMetaModel::getServoDriveConfigs() const
{
	return _servoDriveConfigs;
}

const std::vector<IActuatorConfig::ConstPtr> &CarMetaModel::getMotorConfigs() const
{
	return _motorConfigs;
}

const std::vector<IActuatorConfig::ConstPtr> &CarMetaModel::getLightConfigs() const
{
	return _lightConfigs;
}

const std::vector<IActuatorConfig::ConstPtr> &CarMetaModel::getManeuverStatusConfigs() const
{
	return _statusConfigs;
}
