#pragma once

#include "IAccelerometerPerceptor.h"
#include "ICameraPerceptor.h"
#include "IGyroPerceptor.h"
#include "IIMUPerceptor.h"
#include "IJuryPerceptor.h"
#include "IManeuverListPerceptor.h"
#include "IPerceptor.h"
#include "IPointCloudPerceptor.h"
#include "IValuePerceptor.h"
#include "IWheelTickPerceptor.h"

#include <boost/smart_ptr.hpp>
#include <map>
#include <string>

namespace taco
{
/**
 * General Interface for the perception component.
 * The IPerception provides access to the different perception streams.
 *
 * \author Stefan Glaser
 */
class IPerception
{
  public:
	typedef boost::shared_ptr<IPerception> Ptr;
	typedef boost::shared_ptr<const IPerception> ConstPtr;

	virtual ~IPerception(){};

	/** Retrieve the perceptor instance with the given name.
	 * \return the perceptor with the given name, or NULL, if no such perceptor exists
	 */
	virtual IPerceptor::ConstPtr getPerceptor(std::string name) const = 0;

	/** Retrieve the value-perceptor instance with the given name.
	 * \return the value-perceptor with the given name, or NULL, if no such perceptor exists
	 */
	virtual IDoubleValuePerceptor::ConstPtr getDoubleValuePerceptor(std::string name) const = 0;

	/** Retrieve the camera-perceptor instance with the given name.
	 * \return the camera-perceptor with the given name, or NULL, if no such perceptor exists
	 */
	virtual ICameraPerceptor::ConstPtr getCameraPerceptor(std::string name) const = 0;

	/** Retrieve the depth-camera-perceptor instance with the given name.
	 * \return the depth-camera-perceptor with the given name, or NULL, if no such perceptor exists
	 */
	virtual IPointCloudPerceptor::ConstPtr getPointCloudPerceptor(std::string name) const = 0;

	/** Retrieve the Gyro-perceptor instance with the given name.
	 * \return the Gyro-perceptor with the given name, or NULL, if no such perceptor exists
	 */
	virtual IGyroPerceptor::ConstPtr getGyroPerceptor(std::string name) const = 0;

	/** Retrieve the Accelerometer-perceptor instance with the given name.
	 * \return the Accelerometer-perceptor with the given name, or NULL, if no such perceptor exists
	 */
	virtual IAccelerometerPerceptor::ConstPtr getAccelerometerPerceptor(std::string name) const = 0;

	/** Retrieve the IMU-perceptor instance with the given name.
	 * \return the IMU-perceptor with the given name, or NULL, if no such perceptor exists
	 */
	virtual IIMUPerceptor::ConstPtr getIMUPerceptor(std::string name) const = 0;

	/** Retrieve the WheelTick-perceptor instance with the given name.
	 * \return the WheelTick-perceptor with the given name, or NULL, if no such perceptor exists
	 */
	virtual IWheelTickPerceptor::ConstPtr getWheelTickPerceptor(std::string name) const = 0;

	virtual IJuryPerceptor::ConstPtr getJuryPerceptor(std::string name) const = 0;

	/** Retrieve the ManeuverList-perceptor instance with the given name.
	 * \returns the ManeuverList-perceptor with the given name, or NULL, if no such perceptor exists
	 */
	virtual IManeuverListPerceptor::ConstPtr getManeuverListPerceptor(std::string name) const = 0;

	/** Called to progress with the next perception-map.
	 *  If no new perceptor-map is available the IPerception will stay with the current perceptor-map.
	 * \return true, if the IPerception represents a new perceptor-map, false otherwise
	 */
	virtual const bool nextPerceptorMap() = 0;

	/** Called to notofy the Perception about new perceptions. */
	virtual void updatePerceptors(std::map<std::string, IPerceptor::ConstPtr> &newPerceptors) = 0;
};
}
