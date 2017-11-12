#pragma once

#include "IPerceptor.h"

#include <eigen3/Eigen/Dense>

namespace taco
{
/**
 * Interface for an Accelerometer perceptor.
 *
 * \author Stefan Glaser
 */
class IAccelerometerPerceptor : public virtual IPerceptor
{
  public:
	typedef boost::shared_ptr<IAccelerometerPerceptor> Ptr;
	typedef boost::shared_ptr<const IAccelerometerPerceptor> ConstPtr;

	virtual ~IAccelerometerPerceptor(){};

	/** Retrieve the acceleration-vector of the perceptor.
	 *
	 * \returns The acceleration-vector.
	 */
	virtual const Eigen::Vector3d &getAcceleration() const = 0;
};
}
