#pragma once

#include "IPerceptor.h"
#include <opencv/cv.h>

namespace taco
{
/**
 * Interface for a Camera perceptor.
 *
 * \author Stefan Glaser
 */
class ICameraPerceptor : public virtual IPerceptor
{
  public:
	typedef boost::shared_ptr<ICameraPerceptor> Ptr;
	typedef boost::shared_ptr<const ICameraPerceptor> ConstPtr;

	virtual ~ICameraPerceptor(){};

	virtual const cv::Mat &getImage() const = 0;
};
}
