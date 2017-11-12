#pragma once

#include <boost/smart_ptr.hpp>
#include <string>

namespace taco
{
/**
 * Interface for defining an actuator configuration.
 *
 * \author Stefan Glaser
 */
class IActuatorConfig
{
  public:
	typedef boost::shared_ptr<IActuatorConfig> Ptr;
	typedef boost::shared_ptr<const IActuatorConfig> ConstPtr;

	virtual ~IActuatorConfig(){};

	/** Retrieve the name of the actuator.
	 * \returns the actuator name
	 */
	virtual const std::string &getName() const = 0;

	/** Retrieve the name of the perceptor that corresponds to this actuator.
	 * \returns the corresponding perceptor name
	 */
	virtual const std::string &getPerceptorName() const = 0;

	/** Retrieve the name of the effector that corresponds to this actuator.
	 * \returns the corresponding effector name
	 */
	virtual const std::string &getEffectorName() const = 0;
};
}
