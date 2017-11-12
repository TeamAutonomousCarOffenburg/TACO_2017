#pragma once

#include "utils/geometry/Pose3D.h"

#include <boost/smart_ptr.hpp>
#include <string>

namespace taco
{
/**
 * Interface for defining an abrbitrary sensor configuration.
 *
 * \author Stefan Glaser
 */
class ISensorConfig
{
  public:
	typedef boost::shared_ptr<ISensorConfig> Ptr;
	typedef boost::shared_ptr<const ISensorConfig> ConstPtr;

	virtual ~ISensorConfig(){};

	/** Retrieve the name of the sensor.
	 * \returns the sensor name
	 */
	virtual const std::string &getName() const = 0;

	/** Retrieve the name of the perceptor that corresponds to this sensor.
	 * \returns the corresponding perceptor name
	 */
	virtual const std::string &getPerceptorName() const = 0;

	/** Retrieve the pose where and how this sensor is mounted on the car.
	 * \returns the mounting pose
	 */
	virtual const Pose3D &getPose() const = 0;
};
}
