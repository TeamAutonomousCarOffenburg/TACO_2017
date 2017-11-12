#pragma once

#include "meta/IAxleConfig.h"
#include <utils/geometry/Angle.h>

#include <eigen3/Eigen/Dense>
#include <string>

namespace taco
{
/**
 * The AxleConfig represents an axle of the car.
 *
 * \author Stefan Glaser
 */
class AxleConfig : public virtual IAxleConfig
{
  public:
	AxleConfig(const std::string &name, const Eigen::Vector3d position, const double &length,
			const double &wheelDiameter, const Angle &wheelAngle = Angle::Zero(),
			const std::string &leftWheelTacho = "", const std::string &rightWheelTacho = "");
	virtual ~AxleConfig();

	virtual const std::string &getName() const;
	virtual const Eigen::Vector3d &getPosition() const;
	virtual const double &getLength() const;
	virtual const double &getWheelDiameter() const;
	virtual const Angle &getWheelAngle() const;
	virtual const std::string &getLeftWheelTachoName() const;
	virtual const std::string &getRightWheelTachoName() const;

  private:
	/** The name of the sensor. */
	const std::string _name;

	/** The position of the axle relative to the car. */
	const Eigen::Vector3d _position;

	/** The axle length. */
	const double _length;

	/** The diameter of the wheels mounted to this axle. */
	const double _wheelDiameter;

	/** The angle of the wheels relative to the axle. */
	const Angle _wheelAngle;

	/** The name of the tacho atached to the left wheel. */
	const std::string _leftWheelTacho;

	/** The name of the tacho atached to the right wheel. */
	const std::string _rightWheelTacho;
};
}
