#pragma once

#include "meta/ISensorConfig.h"
#include "utils/geometry/Pose3D.h"

#include <eigen3/Eigen/Dense>

namespace taco
{
/**
 * The SensorConfig represents an arbitrary sensor configuration.
 *
 * \author Stefan Glaser
 */
class SensorConfig : public virtual ISensorConfig
{
  public:
	SensorConfig(const std::string &name, const Eigen::Vector3d &position, const Eigen::AngleAxisd &orientation);
	SensorConfig(const std::string &name, const std::string &perceptorName, const Eigen::Vector3d &position,
			const Eigen::AngleAxisd &orientation);
	virtual ~SensorConfig();

	virtual const std::string &getName() const;
	virtual const std::string &getPerceptorName() const;
	virtual const Pose3D &getPose() const;

  private:
	/** The name of the sensor. */
	std::string _name;

	/** The corresponding perceptor name */
	std::string _perceptorName;

	/** The position where and how the sensor is mounted on the car. */
	Pose3D _pose;
};
}
