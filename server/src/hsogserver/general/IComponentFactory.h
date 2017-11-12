#pragma once

#include "action/IAction.h"
#include "detection/ILaneDetection.h"
#include "detection/IObjectDetection.h"
#include "meta/ICarMetaModel.h"
#include "perception/IPerception.h"
#include "utils/logger/IEventLogger.h"

namespace taco
{
class IComponentFactory
{
  public:
	typedef boost::shared_ptr<IComponentFactory> Ptr;
	typedef boost::shared_ptr<const IComponentFactory> ConstPtr;

	virtual ~IComponentFactory(){};

	virtual ICarMetaModel::Ptr createCarMetaModel() const = 0;

	virtual IAction::Ptr createAction(ICarMetaModel::Ptr carMetaModel) const = 0;

	virtual IPerception::Ptr createPerception(ICarMetaModel::Ptr carMetaModel) const = 0;

	virtual IObjectDetection::Ptr createObjectDetection(ICarMetaModel::Ptr carMetaModel) const = 0;

	virtual ILaneDetection::Ptr createLaneDetection(ICarMetaModel::Ptr carMetaModel) const = 0;

	virtual IEventLogger::Ptr createEventLogger() const = 0;
};
}
