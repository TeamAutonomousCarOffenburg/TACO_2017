#include "ComponentFactory.h"
#include "../meta/impl/JSONCarMetaModel.h"
#include "../meta/impl/JSONCarMetaModelHandler.h"
#include "action/impl/Action.h"
#include "detection/lanedetection/LaneDetection.h"
#include "detection/objectdetection/ObjectDetection.h"
#include "perception/impl/Perception.h"
#include "utils/logger/DummyEventLogger.h"
#include <AADCCar.h>
#include <detection/lanedetection/WideAngleCameraConfig.h>

using namespace taco;

ComponentFactory::ComponentFactory() = default;
ComponentFactory::~ComponentFactory() = default;

ICarMetaModel::Ptr ComponentFactory::createCarMetaModel() const
{
#if TACO_CONFIG == TACO_2017
	std::string configJson = "CarMetaModel2017.json";
#elif TACO_CONFIG == TACO_2016
	std::string configJson = "CarMetaModel2016.json";
#endif

	std::string configPath = string(TACO_DIRECTORY) + "/config/" + configJson;
	JSONCarMetaModelHandler handler(configPath);
	if (!handler.Parse()) {
		std::cout << "Error Parsing " << configPath << std::endl;
	}
	return boost::make_shared<JSONCarMetaModel>(handler.floorHeight, handler.steeringServoName, handler.mainMotorName,
			handler.frontAxle, handler.rearAxle, handler.gyroConfigs, handler.accelerometerConfigs, handler.imuConfigs,
			handler.infraRedConfigs, handler.ultrasonicConfigs, handler.rotationConfigs, handler.voltageConfigs,
			handler.cameraConfigs, handler.depthCameraConfigs, handler.servoDriveConfigs, handler.motorConfigs,
			handler.lightConfigs, handler.statusConfigs);
}

IAction::Ptr ComponentFactory::createAction(ICarMetaModel::Ptr carMetaModel) const
{
	return boost::make_shared<Action>(carMetaModel);
}

IPerception::Ptr ComponentFactory::createPerception(ICarMetaModel::Ptr carMetaModel) const
{
	return boost::make_shared<Perception>();
}

ILaneDetection::Ptr ComponentFactory::createLaneDetection(ICarMetaModel::Ptr carMetaModel) const
{
	ICameraConfig::ConstPtr ccf = carMetaModel->getCameraConfigs()[0];
	LaneDetectionConfiguration ldc;
	// TODO: Fetch more values from camera or meta model
	ldc.width = ccf->getFrameWidth();
	ldc.height = ccf->getFrameHeight();
	ldc.focalPointX = ccf->getFocalPointX();
	ldc.focalPointY = ccf->getFocalPointY();
	ldc.focalHorizontal = ccf->getFocalLengthY();
	ldc.focalVertical = ccf->getFocalLengthX();
#if TACO_CONFIG == TACO_2017
	ldc = WideAngleCameraConfig();
#endif
	return boost::make_shared<LaneDetection>(ldc);
}

IObjectDetection::Ptr ComponentFactory::createObjectDetection(ICarMetaModel::Ptr carMetaModel) const
{
	return boost::make_shared<ObjectDetection>(carMetaModel);
}

IEventLogger::Ptr ComponentFactory::createEventLogger() const
{
	return boost::make_shared<DummyEventLogger>();
}
