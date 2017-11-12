#pragma once

#include "ISensorConfig.h"

#include <boost/smart_ptr.hpp>

namespace taco
{
/**
 * Interface for defining a camera sensor configuration.
 *
 * \author Stefan Glaser
 */
class ICameraConfig : public virtual ISensorConfig
{
  public:
	typedef boost::shared_ptr<ICameraConfig> Ptr;
	typedef boost::shared_ptr<const ICameraConfig> ConstPtr;

	virtual ~ICameraConfig() = default;

	/** Retrieve the width of the camera image.
	 * \returns the camera image width
	 */
	virtual const unsigned int &getFrameWidth() const = 0;

	/** Retrieve the height of the camera image.
	 * \returns the camera image height
	 */
	virtual const unsigned int &getFrameHeight() const = 0;

	virtual const float &getFocalPointX() const = 0;

	virtual const float &getFocalPointY() const = 0;

	/** Retrieve the focal length in x (horizontal) direction of the camera.
	 * \returns the focal length in x direction
	 */
	virtual const float &getFocalLengthX() const = 0;

	/** Retrieve the focal length in y (vertical) direction of the camera.
	 * \returns the focal length in y direction
	 */
	virtual const float &getFocalLengthY() const = 0;
};
}
