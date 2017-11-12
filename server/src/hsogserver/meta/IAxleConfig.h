#pragma once

#include "utils/geometry/Angle.h"

#include <boost/smart_ptr.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <string>

namespace taco
{
/** Interface for defining an axle of the car.
 *
 * \author Stefan Glaser
 */
class IAxleConfig
{
  public:
	typedef boost::shared_ptr<IAxleConfig> Ptr;
	typedef boost::shared_ptr<const IAxleConfig> ConstPtr;

	virtual ~IAxleConfig(){};

	/** Retrieve the name of the sensor.
	 * \returns the sensor name
	 */
	virtual const std::string &getName() const = 0;

	/** Retrieve the axle position relative to the car system.
	 * \returns the axle position
	 */
	virtual const Eigen::Vector3d &getPosition() const = 0;

	/** \brief Retrieve the axle length.
	 *
	 * The axle length is the distance from the left wheel to the right wheel in case of an static axle and
	 * the distance between the rotation points of an steering axle.
	 *
	 * \returns the axle length
	 */
	virtual const double &getLength() const = 0;

	/** Retrieve the diameter of the wheels mounted to this axle.
	 * \returns the wheel diameter
	 */
	virtual const double &getWheelDiameter() const = 0;

	/** Retrieve the z-angle of the (right) wheel relative to the axle.
	 * \returns the (right) wheel angle
	 */
	virtual const Angle &getWheelAngle() const = 0;

	/** Retrieve the name of the tacho sensor attached to the left wheel. */
	virtual const std::string &getLeftWheelTachoName() const = 0;

	/** Retrieve the name of the tacho sensor attached to the right wheel. */
	virtual const std::string &getRightWheelTachoName() const = 0;
};
}
