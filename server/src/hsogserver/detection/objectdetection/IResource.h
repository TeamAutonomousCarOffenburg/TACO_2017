#pragma once

#include "detection/IVisibleObject.h"
#include "meta/ICameraConfig.h"
#include "perception/IPointCloudPerceptor.h"
#include "utils/geometry/AlignedBoundingBox3D.h"
#include <vector>

namespace taco
{
/**
 * Interface: avoidance of cyclic dependencies
 *
 */

class IResource
{
  public:
	virtual void setResult(std::vector<IVisibleObject::Ptr> results) = 0;
	virtual const IPointCloudPerceptor::ConstPtr getPointCloudPerceptor() = 0;
	virtual const ICameraConfig::ConstPtr getPointCloudConfig(std::string name) = 0;
	virtual const std::vector<AlignedBoundingBox3D::Ptr> getDistanceSensorData() = 0;
	virtual const std::vector<AlignedBoundingBox3D::Ptr> getPointCloudSensorData() = 0;
	virtual void setPointCloudSensorData(const std::vector<AlignedBoundingBox3D::Ptr> data) = 0;
};
}
