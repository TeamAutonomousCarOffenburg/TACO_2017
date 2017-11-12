#pragma once

#include "Perceptor.h"
#include "perception/IAccelerometerPerceptor.h"

#include <eigen3/Eigen/Dense>
#include <string>

namespace taco
{
/**
 * The AccelerometerPerceptor class represents a accelerometer perception.
 *
 * \author Stefan Glaser
 */
class AccelerometerPerceptor : public Perceptor, public virtual IAccelerometerPerceptor
{
  public:
	typedef boost::shared_ptr<AccelerometerPerceptor> Ptr;
	typedef boost::shared_ptr<const AccelerometerPerceptor> ConstPtr;

	AccelerometerPerceptor(const std::string &name, const long &time, const double &x, const double &y, const double &z)
		: Perceptor(name, time), _acc(x, y, z){};
	virtual ~AccelerometerPerceptor(){};

	virtual const Eigen::Vector3d &getAcceleration() const
	{
		return _acc;
	};

  protected:
	/** The acceleration vector. */
	Eigen::Vector3d _acc;
};
}
