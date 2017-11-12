#pragma once

#include "IPerceptor.h"

#include "perception/impl/Maneuver.h"
#include <string>

namespace taco
{
/**
 * Interface for an maneuverList perceptor.
 *
 * \author Stefan Glaser
 */
class IManeuverListPerceptor : public virtual IPerceptor
{
  public:
	typedef boost::shared_ptr<IManeuverListPerceptor> Ptr;
	typedef boost::shared_ptr<const IManeuverListPerceptor> ConstPtr;

	virtual ~IManeuverListPerceptor(){};

	/** Retrieve the list of drive instructions.
	 * \retuns the list of drive instructions
	 */
	virtual const std::vector<Maneuver> &getDriveInstructions() const = 0;
};
}
