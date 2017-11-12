#pragma once

#include "CarMetaModel.h"
#include <meta/impl/ConfigFactories/CarMetaModelFactory.h>

namespace taco
{
class JSONCarMetaModel : public CarMetaModel
{
  public:
	JSONCarMetaModel(double _floorHeight, std::string _steeringServoName, std::string _mainMotorName,
			IAxleConfig::ConstPtr _frontAxle, IAxleConfig::ConstPtr _rearAxle,

			std::vector<ISensorConfig::ConstPtr> _gyroConfigs,
			std::vector<ISensorConfig::ConstPtr> _accelerometerConfigs,
			std::vector<ISensorConfig::ConstPtr> _imuConfigs,
			std::vector<IDistanceSensorConfig::ConstPtr> _infraRedConfigs,
			std::vector<IDistanceSensorConfig::ConstPtr> _ultrasonicConfigs,
			std::vector<ISensorConfig::ConstPtr> _rotationConfigs, std::vector<ISensorConfig::ConstPtr> _voltageConfigs,
			std::vector<ICameraConfig::ConstPtr> _cameraConfigs,
			std::vector<ICameraConfig::ConstPtr> _depthCameraConfigs,

			std::vector<IServoDriveConfig::ConstPtr> _servoDriveConfigs,
			std::vector<IActuatorConfig::ConstPtr> _motorConfigs, std::vector<IActuatorConfig::ConstPtr> _lightConfigs,
			std::vector<IActuatorConfig::ConstPtr> _statusConfigs);

	virtual ~JSONCarMetaModel();
};
};
