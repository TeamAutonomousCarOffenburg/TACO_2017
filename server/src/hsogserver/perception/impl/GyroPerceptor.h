#pragma once

#include "Perceptor.h"
#include "perception/IGyroPerceptor.h"

#include <eigen3/Eigen/Dense>
#include <string>

namespace taco
{
/**
 * The GyroPerceptor class represents a gyroscope perception.
 *
 * \author Stefan Glaser
 */
class GyroPerceptor : public Perceptor, public virtual IGyroPerceptor
{
  public:
	typedef boost::shared_ptr<GyroPerceptor> Ptr;
	typedef boost::shared_ptr<const GyroPerceptor> ConstPtr;

	GyroPerceptor(const std::string &name, const long &time, const double &w, const double &x, const double &y,
			const double &z)
		: Perceptor(name, time), _gyro(w, x, y, z){};
	virtual ~GyroPerceptor(){};

	virtual const Eigen::Quaterniond &getGyro() const
	{
		return _gyro;
	};

  protected:
	/** The gyro quaternion. */
	Eigen::Quaterniond _gyro;
};
}
