#pragma once

#include "IAccelerometerPerceptor.h"
#include "IGyroPerceptor.h"
#include "IPerceptor.h"

#include <eigen3/Eigen/Dense>

namespace taco
{
/**
 * Interface for an IMU perceptor.
 *
 * \author Stefan Glaser
 */
class IIMUPerceptor : public virtual IAccelerometerPerceptor, public virtual IGyroPerceptor
{
  public:
	typedef boost::shared_ptr<IIMUPerceptor> Ptr;
	typedef boost::shared_ptr<const IIMUPerceptor> ConstPtr;

	virtual ~IIMUPerceptor(){};
};
}
