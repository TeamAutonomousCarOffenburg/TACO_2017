#include "CameraConfig.h"
#include <boost/concept_check.hpp>

using namespace taco;

CameraConfig::CameraConfig(const std::string &name, const Eigen::Vector3d &position,
		const Eigen::AngleAxisd &orientation, const unsigned int &width, const unsigned int &height,
		const float &focalPointX, const float &focalPointY, const float &focalLengthX, const float &focalLengthY)
	: CameraConfig(
			  name, name, position, orientation, width, height, focalPointX, focalPointY, focalLengthX, focalLengthY)
{
}

CameraConfig::CameraConfig(const std::string &name, const std::string &perceptorName, const Eigen::Vector3d &position,
		const Eigen::AngleAxisd &orientation, const unsigned int &width, const unsigned int &height,
		const float &focalPointX, const float &focalPointY, const float &focalLengthX, const float &focalLengthY)
	: SensorConfig(name, perceptorName, position, orientation), _imageWidth(width), _imageHeight(height),
	  _focalPointX(focalPointX), _focalPointY(focalPointY), _focalLengthX(focalLengthX), _focalLengthY(focalLengthY)
{
}

CameraConfig::~CameraConfig() = default;

const unsigned int &CameraConfig::getFrameWidth() const
{
	return _imageWidth;
}

const unsigned int &CameraConfig::getFrameHeight() const
{
	return _imageHeight;
}

const float &CameraConfig::getFocalPointX() const
{
	return _focalPointX;
}

const float &CameraConfig::getFocalPointY() const
{
	return _focalPointY;
}

const float &CameraConfig::getFocalLengthX() const
{
	return _focalLengthX;
}

const float &CameraConfig::getFocalLengthY() const
{
	return _focalLengthY;
}
