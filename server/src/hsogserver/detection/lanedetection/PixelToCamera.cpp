#include "PixelToCamera.h"
#include "../hsogserver/utils/geometry/Geometry.h"

using namespace taco;
using namespace cv;
using namespace Eigen;

PixelToCamera::PixelToCamera(const Eigen::Vector3d &upVector, const float &focalVertical, const float &focalHorizontal,
		const float &xCenter, const float &yCenter)
	: _upVector(upVector), _focalVertical(focalVertical), _focalHorizontal(focalHorizontal), _xCenter(xCenter),
	  _yCenter(yCenter)
{
}

PixelToCamera::~PixelToCamera()
{
}

Eigen::Vector3d PixelToCamera::pixelToCamera(const cv::Point &pixel) const
{
	// Calculate pixel coordinates relative to the center of the image
	float mx = pixel.x - _xCenter;
	float my = pixel.y - _yCenter;

	// Calculate direction vector from relative pixel coordinates and focal-length
	Vector3d dir(1, -mx / _focalHorizontal, -my / _focalVertical);

	// Rotate the direction vector around the camera twist to get the direction relative to a coordinate system parallel
	// to the car
	Quaterniond cameraRot;
	cameraRot.setFromTwoVectors(_upVector, Vector3d::UnitZ());
	dir = cameraRot * dir;

	// Scale direction vector such that it touches the ground
	return dir * (Geometry::getNorm(_upVector) / std::fabs(dir(2)));
}

cv::Point PixelToCamera::cameraToPixel(const Eigen::Vector3d &direction) const
{
	// Rotate the direction vector around the camera twist
	Quaterniond cameraRot;
	cameraRot.setFromTwoVectors(Vector3d::UnitZ(), _upVector);
	Vector3d dir = cameraRot * direction;

	if (dir(0) == 0) {
		return cv::Point(0, 0);
	}

	double scale = 1.0 / dir(0);

	float mx = dir(1) * scale * _focalHorizontal;
	float my = dir(2) * scale * _focalVertical;

	return Point(-mx + _xCenter, -my + _yCenter);
}
