#pragma once

#include "IVisibleObject.h"

#include <boost/smart_ptr.hpp>

namespace taco
{
/**
 * Base-representation for all visible objects.
 *
 * \author Stefan Glaser
 */
class VisibleObject : public virtual IVisibleObject
{
  public:
	typedef boost::shared_ptr<VisibleObject> Ptr;
	typedef boost::shared_ptr<const VisibleObject> ConstPtr;

	VisibleObject(const std::string &name, const Pose2D &pose, Polygon::Ptr bounds = Polygon::Ptr());
	~VisibleObject();

	virtual void update(const Pose2D &pose);

	virtual const std::string &getName() const;
	virtual const Pose2D &getPose() const;
	virtual void setPose(const Pose2D &pose);
	virtual Polygon::ConstPtr getBoundingPoly() const;
	virtual void setBoundingPoly(Polygon::Ptr bounds);

  protected:
	/** The name of this object. */
	const std::string _name;

	/** The pose of this object. */
	Pose2D _pose;

	/** The bounds of this visible object. */
	Polygon::Ptr _bounds;
};
}
