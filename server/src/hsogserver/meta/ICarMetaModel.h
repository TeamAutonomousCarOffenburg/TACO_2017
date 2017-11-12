#pragma once

#include "IActuatorConfig.h"
#include "IAxleConfig.h"
#include "ICameraConfig.h"
#include "IDistanceSensorConfig.h"
#include "ISensorConfig.h"
#include "IServoDriveConfig.h"

#include <boost/smart_ptr.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>

namespace taco
{
/**
 * Interface for describing the sensors and actuators of a car.
 *
 * \author Stefan Glaser
 */
class ICarMetaModel
{
  public:
	typedef boost::shared_ptr<ICarMetaModel> Ptr;
	typedef boost::shared_ptr<const ICarMetaModel> ConstPtr;

	virtual ~ICarMetaModel(){};

	virtual const std::string &getSteeringServoName() const = 0;

	virtual const std::string &getMainMotorName() const = 0;

	virtual const double &getFloorHeight() const = 0;

	virtual IAxleConfig::ConstPtr getFrontAxle() const = 0;

	virtual IAxleConfig::ConstPtr getRearAxle() const = 0;

	virtual const std::vector<ISensorConfig::ConstPtr> &getGyroConfigs() const = 0;

	virtual const std::vector<ISensorConfig::ConstPtr> &getAccelerometerConfigs() const = 0;

	virtual const std::vector<ISensorConfig::ConstPtr> &getIMUConfigs() const = 0;

	virtual const std::vector<IDistanceSensorConfig::ConstPtr> &getInfraRedConfigs() const = 0;

	virtual const std::vector<IDistanceSensorConfig::ConstPtr> &getUltrasonicConfigs() const = 0;

	virtual const std::vector<ISensorConfig::ConstPtr> &getRotationConfigs() const = 0;

	virtual const std::vector<ISensorConfig::ConstPtr> &getVoltageConfigs() const = 0;

	virtual const std::vector<ICameraConfig::ConstPtr> &getCameraConfigs() const = 0;

	virtual const std::vector<ICameraConfig::ConstPtr> &getDepthCameraConfigs() const = 0;

	virtual const std::vector<IServoDriveConfig::ConstPtr> &getServoDriveConfigs() const = 0;

	virtual const std::vector<IActuatorConfig::ConstPtr> &getMotorConfigs() const = 0;

	virtual const std::vector<IActuatorConfig::ConstPtr> &getLightConfigs() const = 0;

	virtual const std::vector<IActuatorConfig::ConstPtr> &getManeuverStatusConfigs() const = 0;
};
}
