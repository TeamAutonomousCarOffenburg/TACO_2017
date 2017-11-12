#pragma once

#include "Perceptor.h"
#include "perception/IAccelerometerPerceptor.h"

#include <eigen3/Eigen/Dense>
#include <string>

namespace taco
{
/**
 * The IMUPerceptor class represents a IMU perception.
 *
 * \author Stefan Glaser
 */
class IMUPerceptor : public Perceptor, public virtual IIMUPerceptor
{
  public:
	typedef boost::shared_ptr<IMUPerceptor> Ptr;
	typedef boost::shared_ptr<const IMUPerceptor> ConstPtr;

	IMUPerceptor(const std::string &name, const long &time, const double &acc_x, const double &acc_y,
			const double &acc_z, const double &gyro_w, const double &gyro_x, const double &gyro_y, const double &gyro_z)
		: Perceptor(name, time), _acc(acc_x, acc_y, acc_z), _gyro(gyro_w, gyro_x, gyro_y, gyro_z){};
	virtual ~IMUPerceptor(){};

	virtual const Eigen::Vector3d &getAcceleration() const
	{
		return _acc;
	};

	virtual const Eigen::Quaterniond &getGyro() const
	{
		return _gyro;
	};

  protected:
	/** The acceleration vector. */
	Eigen::Vector3d _acc;

	/** The gyro quaternion. */
	Eigen::Quaterniond _gyro;
};
}
