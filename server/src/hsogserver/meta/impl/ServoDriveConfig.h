#pragma once

#include "ActuatorConfig.h"
#include "meta/IServoDriveConfig.h"

namespace taco
{
/**
 * The ServoDriveConfig represents a ServoDrive actuator configuration.
 *
 * \author Stefan Glaser
 */
class ServoDriveConfig : public ActuatorConfig, public virtual IServoDriveConfig
{
  public:
	ServoDriveConfig(const std::string &name, const double &min, const double &max);
	ServoDriveConfig(const std::string &name, const std::string &perceptorName, const std::string &effectorName,
			const double &min, const double &max);
	virtual ~ServoDriveConfig();

	virtual const double &getMinPosition() const;
	virtual const double &getMaxPosition() const;

  private:
	/** The minimum possible position. */
	double _minPosition;

	/** The maximum possible position. */
	double _maxPosition;
};
}
