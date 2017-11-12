#include "PointCloudWorker.h"

#define FRAMES_PER_SECOND 2

typedef std::chrono::duration<int, std::ratio<1, FRAMES_PER_SECOND>> frame_duration;

using namespace taco;
using namespace pcl;

PointCloudWorker::PointCloudWorker(IResource *resource) : _resource(resource)
{
}

PointCloudWorker::~PointCloudWorker()
{
}

void PointCloudWorker::run()
{
	while (!m_stop) {
		auto start_time = std::chrono::steady_clock::now();
		auto end_time = start_time + frame_duration(1);
		IPointCloudPerceptor::ConstPtr perceptor = _resource->getPointCloudPerceptor();
		if (perceptor) {
			PointCloud<PointXYZ>::Ptr cloud = perceptor->getPointCloud();
			if (cloud && !cloud->empty()) {
				cloud = PCLUtils::downSample(cloud, 0.05);
				cloud = PCLUtils::extractFloor(cloud, true);
				if (!cloud->empty()) {
					cloud = PCLUtils::removeOutliers(cloud, 10);
					std::vector<PointIndices> clusters = PCLUtils::segmentation(cloud, 10, 10);
					std::vector<AlignedBoundingBox3D::Ptr> result;

					for (PointIndices cluster : clusters) {
						ExtractIndices<PointXYZ> extract;
						PointCloud<PointXYZ>::Ptr extractedCloudPtr(new PointCloud<PointXYZ>());
						extract.setInputCloud(cloud);
						PointIndices::Ptr indicesPtr = PointIndices::Ptr(new PointIndices(cluster));
						extract.setIndices(indicesPtr);
						extract.setNegative(false);
						extract.filter(*extractedCloudPtr);
						AlignedBoundingBox3D::Ptr box = PCLUtils::alignedBoundingBox(extractedCloudPtr);
						Pose3D cameraPose = _resource->getPointCloudConfig(perceptor->getName())->getPose();
						AlignedBoundingBox3D localBox = cameraPose * *box;
						result.push_back(boost::make_shared<AlignedBoundingBox3D>(localBox));
					}
					_resource->setPointCloudSensorData(result);
				}
			}
		}
		// Sleep if necessary
		std::this_thread::sleep_until(end_time);
	}
}
