#pragma once

#include "IVisibleObject.h"
#include "utils/geometry/Pose2D.h"
#include "utils/roadsigns/ISignConstants.h"

#include <boost/smart_ptr.hpp>

namespace taco
{
class IRoadSign : public virtual IVisibleObject
{
  public:
	typedef boost::shared_ptr<IRoadSign> Ptr;
	typedef boost::shared_ptr<const IRoadSign> ConstPtr;

	virtual ~IRoadSign(){};

	virtual const SignType &getSignType() const = 0;
};
}
