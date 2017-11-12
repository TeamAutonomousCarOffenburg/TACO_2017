#pragma once

#include "IPerceptor.h"
using namespace taco;
namespace taco
{
class IRoadSignPerceptor : public virtual IPerceptor
{
  public:
	typedef boost::shared_ptr<IRoadSignPerceptor> Ptr;
	typedef boost::shared_ptr<const IRoadSignPerceptor> ConstPtr;

	virtual ~IRoadSignPerceptor(){};

	virtual const int getId() const = 0;
	virtual const float getImageSize() const = 0;
	virtual const cv::Mat getTVec() const = 0;
	virtual const cv::Mat getRVec() const = 0;
};
}
