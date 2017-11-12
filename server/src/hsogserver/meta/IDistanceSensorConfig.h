#pragma once

#include "ISensorConfig.h"

#include <boost/smart_ptr.hpp>

namespace taco
{
/**
 * Interface for defining a distance sensor configuration.
 *
 * \author Stefan Glaser
 */
class IDistanceSensorConfig : public virtual ISensorConfig
{
  public:
	typedef boost::shared_ptr<IDistanceSensorConfig> Ptr;
	typedef boost::shared_ptr<const IDistanceSensorConfig> ConstPtr;

	virtual ~IDistanceSensorConfig(){};

	/** Retrieve the minimum distance this sensor can measure.
	 * \returns the minimum distance
	 */
	virtual const double &getMinDistance() const = 0;

	/** Retrieve the maximum distance this sensor can measure.
	 * \returns the maximum distance
	 */
	virtual const double &getMaxDistance() const = 0;

	/** Retrieve the width of the distance scan.
	 * \returns the scan width
	 */
	virtual const double &getScanWidth() const = 0;
};
}
