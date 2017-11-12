#pragma once

#include "meta/ICarMetaModel.h"

#include <vector>

namespace taco
{
/**
 * The CarMetaModel descibes the sensors and actuator mounted on the car.
 *
 * \author Stefan Glaser
 */
class CarMetaModel : public virtual ICarMetaModel
{
  public:
	CarMetaModel(const double &floorHeight, const std::string &steeringServoName, const std::string &mainMotorName);
	virtual ~CarMetaModel();

	virtual const std::string &getSteeringServoName() const;
	virtual const std::string &getMainMotorName() const;
	virtual const double &getFloorHeight() const;
	virtual IAxleConfig::ConstPtr getFrontAxle() const;
	virtual IAxleConfig::ConstPtr getRearAxle() const;

	virtual const std::vector<ISensorConfig::ConstPtr> &getGyroConfigs() const;
	virtual const std::vector<ISensorConfig::ConstPtr> &getAccelerometerConfigs() const;
	virtual const std::vector<ISensorConfig::ConstPtr> &getIMUConfigs() const;
	virtual const std::vector<IDistanceSensorConfig::ConstPtr> &getInfraRedConfigs() const;
	virtual const std::vector<IDistanceSensorConfig::ConstPtr> &getUltrasonicConfigs() const;
	virtual const std::vector<ISensorConfig::ConstPtr> &getRotationConfigs() const;
	virtual const std::vector<ISensorConfig::ConstPtr> &getVoltageConfigs() const;
	virtual const std::vector<ICameraConfig::ConstPtr> &getCameraConfigs() const;
	virtual const std::vector<ICameraConfig::ConstPtr> &getDepthCameraConfigs() const;
	virtual const std::vector<IServoDriveConfig::ConstPtr> &getServoDriveConfigs() const;
	virtual const std::vector<IActuatorConfig::ConstPtr> &getMotorConfigs() const;
	virtual const std::vector<IActuatorConfig::ConstPtr> &getLightConfigs() const;
	virtual const std::vector<IActuatorConfig::ConstPtr> &getManeuverStatusConfigs() const;

  protected:
	const double _floorHeight;
	const std::string _steeringServoName;
	const std::string _mainMotorName;
	IAxleConfig::ConstPtr _frontAxle;
	IAxleConfig::ConstPtr _rearAxle;

	std::vector<ISensorConfig::ConstPtr> _gyroConfigs;
	std::vector<ISensorConfig::ConstPtr> _accelerometerConfigs;
	std::vector<ISensorConfig::ConstPtr> _imuConfigs;
	std::vector<IDistanceSensorConfig::ConstPtr> _infraRedConfigs;
	std::vector<IDistanceSensorConfig::ConstPtr> _ultrasonicConfigs;
	std::vector<ISensorConfig::ConstPtr> _rotationConfigs;
	std::vector<ISensorConfig::ConstPtr> _voltageConfigs;
	std::vector<ICameraConfig::ConstPtr> _cameraConfigs;
	std::vector<ICameraConfig::ConstPtr> _depthCameraConfigs;

	std::vector<IServoDriveConfig::ConstPtr> _servoDriveConfigs;
	std::vector<IActuatorConfig::ConstPtr> _motorConfigs;
	std::vector<IActuatorConfig::ConstPtr> _lightConfigs;
	std::vector<IActuatorConfig::ConstPtr> _statusConfigs;
};
}
