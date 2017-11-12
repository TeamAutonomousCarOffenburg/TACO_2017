#pragma once

#include "Perceptor.h"
#include "perception/IPointCloudPerceptor.h"

#include <string>

namespace taco
{
/**
 * The PointCloudPerceptor class represents a 3D point cloud.
 *
 * \author Stefan Glaser
 */
class PointCloudPerceptor : public Perceptor, public virtual IPointCloudPerceptor
{
  public:
	typedef boost::shared_ptr<PointCloudPerceptor> Ptr;
	typedef boost::shared_ptr<const PointCloudPerceptor> ConstPtr;

	PointCloudPerceptor(const std::string &name, const long &time, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
		: Perceptor(name, time), _cloud(cloud){};
	virtual ~PointCloudPerceptor(){};

	virtual pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud() const
	{
		return _cloud;
	};

  protected:
	/** The point cloud. */
	pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
};
}
