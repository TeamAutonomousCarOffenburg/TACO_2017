#pragma once

#include "Perceptor.h"
#include "perception/ICameraPerceptor.h"

#include <string>

namespace taco
{
/**
 * The CameraPerceptor class represents a camera perception.
 *
 * \author Stefan Glaser
 */
class CameraPerceptor : public Perceptor, public virtual ICameraPerceptor
{
  public:
	typedef boost::shared_ptr<CameraPerceptor> Ptr;
	typedef boost::shared_ptr<const CameraPerceptor> ConstPtr;

	CameraPerceptor(const std::string &name, const long &time, cv::Mat image) : Perceptor(name, time), _image(image){};
	virtual ~CameraPerceptor(){};

	virtual const cv::Mat &getImage() const
	{
		return _image;
	};

  protected:
	cv::Mat _image;
};
}
