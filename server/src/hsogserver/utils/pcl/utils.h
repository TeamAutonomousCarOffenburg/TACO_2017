#pragma once
#include "../geometry/AlignedBoundingBox3D.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace taco
{
/** \brief Utility class for point cloud calculations in 3D space.
 *
 * \author Peter Walden
 */
class PCLUtils
{
  public:
	/** Retrieve a down sampled cloud data by the use of voxel grid filter.
	 * \param pointCloud: input point cloud
	 * \param leafSize: in m, used for x,y,z axis
	 *
	 * \return down sampled point cloud
	 */
	static pcl::PointCloud<pcl::PointXYZ>::Ptr downSample(
			pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud, float leafSize)
	{
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud(pointCloud);
		sor.setLeafSize(leafSize, leafSize, leafSize);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		sor.filter(*cloud_filtered);
		return cloud_filtered;
	}

	/** extract floor
	 * \param pointCloud: input point cloud
	 * \param negative: if true:  point cloud without floor will be return
	 *                    false: returned point cloud contains floor only
	 *
	 * \return processed point cloud, empty if no planar model could be estimated from given cloud.
	 */
	static pcl::PointCloud<pcl::PointXYZ>::Ptr extractFloor(
			pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud, bool negative)
	{
		// Segment the largest planar component
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(500);
		seg.setDistanceThreshold(0.01);
		seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, -1.0f));
		seg.setEpsAngle(M_PI * 0.1);
		seg.setInputCloud(pointCloud);

		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0) {
			std::cerr << "PCLUtils: Could not estimate a planar model for the given dataset." << std::endl;
			return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
		}
		// Extract the inliers
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(pointCloud);
		extract.setIndices(inliers);
		extract.setNegative(negative);
		pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
		extract.filter(*result);
		return result;
	}

	/** remove outlier points
	 * \param cloud: input point cloud
	 * \param meanK: the number of nearest neighbors to use for mean distance estimation.
	 *
	 * \return processed point cloud
	 */
	static pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double meanK)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		// Create the filtering object
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(cloud);
		sor.setMeanK(meanK);
		sor.setStddevMulThresh(0.5);
		sor.filter(*cloud_filtered);
		return cloud_filtered;
	}

	/** find clusters in point cloud and group them.
	 * \param cloud: input point cloud
	 * \param minClusterSize: smalest number of point that can form a clusters
	 * \param numberOfNeighbours: number of neighbors that are considered
	 *
	 * \return vector of point indices that represents a clusterj
	 */
	static std::vector<pcl::PointIndices> segmentation(
			pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double minClusterSize, double numberOfNeighbours)
	{
		// create empty normal objects to use region growing without respecting the point normals
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
		for (uint i = 0; i < cloud->size(); i++) {
			normals->push_back(pcl::Normal());
		}
		pcl::search::Search<pcl::PointXYZ>::Ptr tree =
				boost::shared_ptr<pcl::search::Search<pcl::PointXYZ>>(new pcl::search::KdTree<pcl::PointXYZ>);

		// use region growing without normals
		pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
		reg.setMinClusterSize(minClusterSize);
		reg.setMaxClusterSize(1000);
		reg.setSearchMethod(tree);
		reg.setNumberOfNeighbours(numberOfNeighbours);
		reg.setInputCloud(cloud);
		reg.setInputNormals(normals);
		// do not respect normals
		reg.setSmoothnessThreshold(M_PI);
		reg.setCurvatureThreshold(1.0);

		std::vector<pcl::PointIndices> clusters;
		reg.extract(clusters);
		return clusters;
	}

	/** calculations for 3d aligned bounding box
	 * \param cloud: input point cloud
	 *
	 * \return AlignedBoundingBox3D
	 */
	static AlignedBoundingBox3D::Ptr alignedBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
	{
		pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
		feature_extractor.setInputCloud(cloud);
		feature_extractor.compute();

		pcl::PointXYZ min_point_AABB;
		pcl::PointXYZ max_point_AABB;
		Eigen::Vector3f mass_center;
		feature_extractor.getAABB(min_point_AABB, max_point_AABB);
		feature_extractor.getMassCenter(mass_center);
		Eigen::Vector3d origin = mass_center.cast<double>();
		Eigen::Vector3d minPoint = Eigen::Vector3d(min_point_AABB.getArray3fMap().cast<double>()) - origin;
		Eigen::Vector3d maxPoint = Eigen::Vector3d(max_point_AABB.getArray3fMap().cast<double>()) - origin;

		AlignedBoundingBox3D::Ptr result(new AlignedBoundingBox3D(origin, minPoint, maxPoint));
		return result;
	}
};
}
