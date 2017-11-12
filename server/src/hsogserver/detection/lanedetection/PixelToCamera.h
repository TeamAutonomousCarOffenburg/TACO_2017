#pragma once

#include <boost/smart_ptr.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>

namespace taco
{
/** \brief Helper class providing transformations from image coordinates to camera and/or world coordinates.
 *
 * This class realizes a ray-backtracing. Image coordinates are first converted into a direction vector.
 * This direction vector is then corrected by the actual rotation of the camera to match the car coordinate system axes.
 * After that, the corrected direction is traced back to the point where it intersects the bottom plane.
 *
 * \author Stefan Glaser
 * \author Andreas Saelinger
 */
class PixelToCamera
{
  public:
	typedef boost::shared_ptr<PixelToCamera> Ptr;
	typedef boost::shared_ptr<const PixelToCamera> ConstPtr;

	PixelToCamera(const Eigen::Vector3d &upVector, const float &focalVertical, const float &focalHorizontal,
			const float &xCenter, const float &yCenter);

	~PixelToCamera();

	/**
	 * Converts the passed points pixel coordinates to 3D camera coordinate system
	 */
	Eigen::Vector3d pixelToCamera(const cv::Point &pixel) const;

	/**
	 * Converts the passed direction vector to pixel coordinates
	 */
	cv::Point cameraToPixel(const Eigen::Vector3d &direction) const;

  private:
	Eigen::Vector3d _upVector;
	float _focalVertical;
	float _focalHorizontal;
	float _xCenter;
	float _yCenter; // yCenter should be 0 because we already use only the lower half of the image
};
}
