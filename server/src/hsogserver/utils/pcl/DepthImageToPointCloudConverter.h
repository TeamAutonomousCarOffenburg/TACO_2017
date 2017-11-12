#pragma once

// point cloud lib header
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// opencv header
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace taco
{
/**
 * Converter for converting a depth image to a point cloud.
 *
 * \author Peter Walden
 */
class DepthImageToPointCloudConverter
{
  public:
	static void convert(cv::Mat *depthImage, double focalLengthX, double focalLengthY,
			pcl::PointCloud<pcl::PointXYZ>::Ptr &outputCloud)
	{
		pcl::PointXYZ point;

		cv::Size matSize = depthImage->size();

		int height = matSize.height;
		int width = matSize.width;

		// default optical center
		float cv = (width / 2.0) - 0.5;
		float cu = (height / 2.0) - 0.5;

		for (int u = 0; u < height; u++) {
			for (int v = 0; v < width; v++) {
				// get depth value from pixel
				uint16_t depthValue = depthImage->at<uint16_t>(u, v);
				// drop 0 value and points over 4 meter
				if (depthValue > 0 && depthValue < 4000) {
					// convert gray value in distance (in m)
					float distance = depthValue / 1000.0;
					// translation from depth pixel (u,v,d) to a point (x,y,z)
					point.z = (u - cu) * distance / focalLengthY * -1;
					point.y = (v - cv) * distance / focalLengthX * -1;
					point.x = distance;

					outputCloud->push_back(point);
				}
			}
		}
	}
};
}
