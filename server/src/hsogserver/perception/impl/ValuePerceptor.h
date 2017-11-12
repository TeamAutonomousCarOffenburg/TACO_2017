#pragma once

#include "Perceptor.h"
#include "perception/IValuePerceptor.h"

#include <string>

namespace taco
{
/**
 * The ValuePerceptor class represents a simple value perception.
 *
 * \author Stefan Glaser
 */
template <typename T> class ValuePerceptor : public Perceptor, public virtual IValuePerceptor<T>
{
  public:
	typedef boost::shared_ptr<ValuePerceptor<T>> Ptr;
	typedef boost::shared_ptr<const ValuePerceptor<T>> ConstPtr;

	ValuePerceptor(const std::string &name, const long &time, const T &value) : Perceptor(name, time), _value(value){};
	virtual ~ValuePerceptor(){};

	/** Retrieve the perceptor value.
	 *
	 * \returns the perceptor value
	 */
	virtual const T &getValue() const
	{
		return _value;
	};

  protected:
	/** The value of the perceptor. */
	T _value;
};

typedef ValuePerceptor<bool> BoolValuePerceptor;
typedef ValuePerceptor<int> IntValuePerceptor;
typedef ValuePerceptor<double> DoubleValuePerceptor;
}
