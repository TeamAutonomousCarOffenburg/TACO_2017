#pragma once

#include "IComponentFactory.h"

namespace taco
{
/**
 * The ComponentFactory is responsible for providing all concrete component instances during initialization.
 *
 * \author Stefan Glaser
 */
class ComponentFactory : public virtual IComponentFactory
{
  public:
	ComponentFactory();
	virtual ~ComponentFactory();

	virtual ICarMetaModel::Ptr createCarMetaModel() const;

	virtual IAction::Ptr createAction(ICarMetaModel::Ptr carMetaModel) const;

	virtual IPerception::Ptr createPerception(ICarMetaModel::Ptr carMetaModel) const;

	virtual ILaneDetection::Ptr createLaneDetection(ICarMetaModel::Ptr carMetaModel) const;

	virtual IObjectDetection::Ptr createObjectDetection(ICarMetaModel::Ptr carMetaModel) const;

	virtual IEventLogger::Ptr createEventLogger() const;
};
}
