#pragma once

#include <cv.h>
#include <string>

#include "Perceptor.h"
#include "perception/IRoadSignPerceptor.h"

namespace taco
{
class RoadSignPerceptor : public Perceptor, public virtual IRoadSignPerceptor
{
  public:
	typedef boost::shared_ptr<RoadSignPerceptor> Ptr;
	typedef boost::shared_ptr<const RoadSignPerceptor> ConstPtr;

	RoadSignPerceptor(const std::string &name, const long &time, const int &id, const float &imageSize,
			const cv::Mat tVec, const cv::Mat rVec)
		: Perceptor(name, time), _id(id), _imageSize(imageSize), _tVec(tVec), _rVec(rVec){};
	virtual ~RoadSignPerceptor(){};

	virtual const int getId() const
	{
		return _id;
	}

	virtual const float getImageSize() const
	{
		return _imageSize;
	}

	virtual const cv::Mat getTVec() const
	{
		return _tVec;
	}

	virtual const cv::Mat getRVec() const
	{
		return _rVec;
	}

  protected:
	int _id = 0;
	float _imageSize = 0;
	cv::Mat _tVec;
	cv::Mat _rVec;
};
}
