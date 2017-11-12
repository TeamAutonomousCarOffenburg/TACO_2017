#pragma once

#include "IPerceptor.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace taco
{
/**
 * Interface for a Depth Camera (RGB-D) perceptor.
 *
 * \author Stefan Glaser
 */
class IPointCloudPerceptor : public virtual IPerceptor
{
  public:
	typedef boost::shared_ptr<IPointCloudPerceptor> Ptr;
	typedef boost::shared_ptr<const IPointCloudPerceptor> ConstPtr;

	virtual ~IPointCloudPerceptor(){};

	virtual pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud() const = 0;
};
}
