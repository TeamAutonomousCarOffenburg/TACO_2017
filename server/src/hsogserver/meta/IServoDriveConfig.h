#pragma once

#include "IActuatorConfig.h"

#include <boost/smart_ptr.hpp>

namespace taco
{
/**
 * Interface for defining a ServoDrive actuator configuration.
 *
 * \author Stefan Glaser
 */
class IServoDriveConfig : public virtual IActuatorConfig
{
  public:
	typedef boost::shared_ptr<IServoDriveConfig> Ptr;
	typedef boost::shared_ptr<const IServoDriveConfig> ConstPtr;

	virtual ~IServoDriveConfig(){};

	/** Retrieve the minimum possible servo position.
	 * \returns the minimum servo position
	 */
	virtual const double &getMinPosition() const = 0;

	/** Retrieve the maximum possible servo position.
	 * \returns the maximum servo position
	 */
	virtual const double &getMaxPosition() const = 0;
};
}
