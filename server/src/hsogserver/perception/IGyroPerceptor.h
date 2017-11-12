#pragma once

#include "IPerceptor.h"

#include <eigen3/Eigen/Dense>

namespace taco
{
/**
 * Interface for an Gyroscope perceptor.
 *
 * \author Stefan Glaser
 */
class IGyroPerceptor : public virtual IPerceptor
{
  public:
	typedef boost::shared_ptr<IGyroPerceptor> Ptr;
	typedef boost::shared_ptr<const IGyroPerceptor> ConstPtr;

	virtual ~IGyroPerceptor(){};

	/** Retrieve the gyro quaternion.
	 *
	 * \returns The gyro quaternion.
	 */
	virtual const Eigen::Quaterniond &getGyro() const = 0;
};
}
