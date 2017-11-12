#pragma once

#include <boost/smart_ptr.hpp>
#include <string>
#include <utils/geometry/Polygon.h>
#include <utils/geometry/Pose2D.h>

namespace taco
{
/**
 * General Interface for all visible objects.
 *
 * \author Stefan Glaser
 */
class IVisibleObject
{
  public:
	typedef boost::shared_ptr<IVisibleObject> Ptr;
	typedef boost::shared_ptr<const IVisibleObject> ConstPtr;

	~IVisibleObject(){};

	/** Retrieve the name of this visible object.
	 * \returns the name of this object
	 */
	virtual const std::string &getName() const = 0;

	/** Retrieve the pose of this visible object.
	 * \returns the pose of this object
	 */
	virtual const Pose2D &getPose() const = 0;

	/** Retrieve the surrounding polygon of this visual object. */
	virtual Polygon::ConstPtr getBoundingPoly() const = 0;
};
}
