#include "FloorNormalDetection.h"
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "perception/IPointCloudPerceptor.h"
#include "utils/pcl/utils.h"

#include <chrono>

using namespace taco;
using namespace Eigen;
using namespace std;

FloorNormalDetection::FloorNormalDetection(double defaultCameraHeight)
{
	// init buffer and set size
	_floorNormals = boost::circular_buffer<Vector3d>();
	_floorNormals.set_capacity(15);
	// push back initial vector with default camera height from meta model
	_floorNormals.push_back(Vector3d(0, 0, defaultCameraHeight));
	_lastCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pcl::PointCloud<pcl::PointXYZ>());
}

Vector3d FloorNormalDetection::calculateFloorNormal(IPointCloudPerceptor::ConstPtr pointCloudPerceptor)
{
	static long lastMeasurement;
	if (!pointCloudPerceptor) {
		std::cout << "err" << std::endl;
	}
	long measurement = pointCloudPerceptor->getTime();
	if (lastMeasurement == measurement)
		return getAverageOfLastMeasurements();

	lastMeasurement = measurement;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = pointCloudPerceptor->getPointCloud();
	if (!pointCloud || pointCloud->size() == 0) {
		// got empty cloud from sensor
		return getAverageOfLastMeasurements();
	}
	pointCloud = PCLUtils::downSample(pointCloud, 0.3f);
	pcl::PointCloud<pcl::PointXYZ>::Ptr floor = PCLUtils::extractFloor(pointCloud, false);
	if (floor->size() == 0) {
		// could not estimate floor
		return getAverageOfLastMeasurements();
	}
	_lastCloud = floor;
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(floor);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	// Use all neighbors in a sphere of radius 400m (=^ whole point cloud in this case)
	ne.setRadiusSearch(400);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	// Compute the features
	ne.compute(*cloud_normals);
	if (cloud_normals->size() > 0) {
		pcl::Normal normal = cloud_normals->front();
		Vector3d floorNormal = Vector3d(normal.normal_x, normal.normal_y, normal.normal_z);
		Vector4f floorCentroid(0.0, 0.0, 0.0, 0.0);
		pcl::compute3DCentroid(*floor, floorCentroid);
		// calculate distance from camera to floor and adjust floor normal vector
		double distance =
				(floorCentroid(0) * floorNormal(0) + floorCentroid(1) * floorNormal(1) +
						floorCentroid(2) * floorNormal(2)) /
				(floorNormal(0) * floorNormal(0) + floorNormal(1) * floorNormal(1) + floorNormal(2) * floorNormal(2));
		_floorNormals.push_back(fabs(distance) * floorNormal);
	} else {
		// floor normal calculation failed
	}
	return getAverageOfLastMeasurements();
}

Vector3d FloorNormalDetection::getAverageOfLastMeasurements()
{
	Vector3d vectorSum = Vector3d(0.0, 0.0, 0.0);
	int size = _floorNormals.size();
	for (int i = 0; i < size; i++) {
		vectorSum = vectorSum + _floorNormals.at(i);
	}
	return vectorSum / size;
}
