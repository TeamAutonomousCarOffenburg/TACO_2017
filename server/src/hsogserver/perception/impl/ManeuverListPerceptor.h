#pragma once

#include <perception/IPerception.h>
#include <string>

#include "Maneuver.h"
#include "Perceptor.h"
#include "perception/IManeuverListPerceptor.h"

namespace taco
{
/**
 * The ManeuverListPerceptor class represents a maneuver list perception.
 *
 * \author Stefan Glaser
 */
class ManeuverListPerceptor : public Perceptor, public virtual IManeuverListPerceptor
{
  public:
	typedef boost::shared_ptr<ManeuverListPerceptor> Ptr;
	typedef boost::shared_ptr<const ManeuverListPerceptor> ConstPtr;

	ManeuverListPerceptor(const std::string &name, const long &time, const std::vector<Maneuver> &driveInstructions)
		: Perceptor(name, time), _driveInstructions(driveInstructions){

								 };
	virtual ~ManeuverListPerceptor(){};

	virtual const std::vector<Maneuver> &getDriveInstructions() const
	{
		return _driveInstructions;
	}

  protected:
	std::vector<Maneuver> _driveInstructions;
};
}
