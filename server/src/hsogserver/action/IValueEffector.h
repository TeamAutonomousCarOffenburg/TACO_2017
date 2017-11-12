#pragma once

#include "IEffector.h"

#include <boost/smart_ptr.hpp>

namespace taco
{
/**
 * Interface for a simple single value effector.
 *
 * \author Stefan Glaser
 */
template <typename T> class IValueEffector : public virtual IEffector
{
  public:
	typedef boost::shared_ptr<IValueEffector> Ptr;
	typedef boost::shared_ptr<const IValueEffector> ConstPtr;

	virtual ~IValueEffector(){};

	virtual void setValue(const T &newValue, const long &time) = 0;
	virtual const T &getValue() const = 0;
};

typedef IValueEffector<bool> IBoolValueEffector;
typedef IValueEffector<int> IIntValueEffector;
typedef IValueEffector<double> IDoubleValueEffector;
}
