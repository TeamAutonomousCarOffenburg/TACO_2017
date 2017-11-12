#pragma once

#include "IVisibleObject.h"
#include "perception/IPointCloudPerceptor.h"
#include "perception/IValuePerceptor.h"
#include <boost/smart_ptr.hpp>
#include <vector>

namespace taco
{
// class IWorldModel;

/**
 * Interface for the object detection.
 *
 */
class IObjectDetection
{
  public:
	virtual ~IObjectDetection(){};
	virtual void stop() = 0;
	virtual void start() = 0;
	virtual void update() = 0;

	virtual void setData(IPointCloudPerceptor::ConstPtr pointCloudPerceptor,
			std::vector<IDoubleValuePerceptor::ConstPtr> perceptorsIR,
			std::vector<IDoubleValuePerceptor::ConstPtr> perceptorsUS) = 0;
	/** Retrieve the most recent result from object detection.
	 * \return vector of objects once, on second call empty vector till new data is available.
	 */
	virtual std::vector<IVisibleObject::Ptr> getObjectResult() = 0;

	typedef boost::shared_ptr<IObjectDetection> Ptr;
};
}
