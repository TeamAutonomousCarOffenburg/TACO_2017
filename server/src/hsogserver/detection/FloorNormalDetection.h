#pragma once

#include "perception/IPointCloudPerceptor.h"
#include <boost/circular_buffer.hpp>
#include <eigen3/Eigen/Core>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

namespace taco
{
class FloorNormalDetection
{
  public:
	FloorNormalDetection(double defaultCameraHeight);
	Eigen::Vector3d calculateFloorNormal(IPointCloudPerceptor::ConstPtr pointCloudPerceptor);
	pcl::PointCloud<pcl::PointXYZ>::Ptr _lastCloud;

  private:
	boost::circular_buffer<Eigen::Vector3d> _floorNormals;
	Eigen::Vector3d getAverageOfLastMeasurements();
};
}
