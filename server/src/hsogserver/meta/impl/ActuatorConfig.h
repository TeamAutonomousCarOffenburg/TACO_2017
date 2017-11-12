#pragma once

#include "meta/IActuatorConfig.h"

namespace taco
{
/**
 * The ActuatorConfig represents an arbitrary actuator configuration.
 *
 * \author Stefan Glaser
 */
class ActuatorConfig : public virtual IActuatorConfig
{
  public:
	ActuatorConfig(const std::string &name);
	ActuatorConfig(const std::string &name, const std::string &perceptorName, const std::string &effectorName);
	virtual ~ActuatorConfig();

	virtual const std::string &getName() const;
	virtual const std::string &getPerceptorName() const;
	virtual const std::string &getEffectorName() const;

  private:
	/** The name of the actuator. */
	std::string _name;

	/** The corresponding perceptor name */
	std::string _perceptorName;

	/** The corresponding perceptor name */
	std::string _effectorName;
};
}
