#pragma once

#include "IPerceptor.h"

namespace taco
{
/**
 * Interface for a simple value perceptor.
 *
 * \author Stefan Glaser
 */
template <typename T> class IValuePerceptor : public virtual IPerceptor
{
  public:
	typedef boost::shared_ptr<IValuePerceptor<T>> Ptr;
	typedef boost::shared_ptr<const IValuePerceptor<T>> ConstPtr;

	virtual ~IValuePerceptor(){};

	/** Retrieve the value of the perceptor.
	 *
	 * \returns The perceptor value.
	 */
	virtual const T &getValue() const = 0;
};

typedef IValuePerceptor<bool> IBoolValuePerceptor;
typedef IValuePerceptor<int> IIntValuePerceptor;
typedef IValuePerceptor<double> IDoubleValuePerceptor;
}
