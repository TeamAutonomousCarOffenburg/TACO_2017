#pragma once

#include "SensorConfig.h"
#include "meta/IDistanceSensorConfig.h"

namespace taco
{
/**
 * The DistanceSensorConfig represents an ultrasonic sensor configuration.
 *
 * \author Stefan Glaser
 */
class DistanceSensorConfig : public SensorConfig, public virtual IDistanceSensorConfig
{
  public:
	DistanceSensorConfig(const std::string &name, const Eigen::Vector3d &position, const Eigen::AngleAxisd &orientation,
			const double &minDistance, const double &maxDistance, const double &scanWidth);
	DistanceSensorConfig(const std::string &name, const std::string &perceptorName, const Eigen::Vector3d &position,
			const Eigen::AngleAxisd &orientation, const double &minDistance, const double &maxDistance,
			const double &scanWidth);
	virtual ~DistanceSensorConfig();

	virtual const double &getMinDistance() const;
	virtual const double &getMaxDistance() const;
	virtual const double &getScanWidth() const;

  private:
	/** The minimum distance this sensor is able to measure properly. */
	double _minDistance;

	/** The maximum distance this sensor is able to measure properly. */
	double _maxDistance;

	/** The scan with of this distance sensor. */
	double _scanWidth;
};
}
