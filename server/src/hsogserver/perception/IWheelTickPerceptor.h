#pragma once

#include "IPerceptor.h"

namespace taco
{
/**
 * Interface for a wheel tick perceptor.
 *
 * \author Stefan Glaser
 */
class IWheelTickPerceptor : public virtual IPerceptor
{
  public:
	typedef boost::shared_ptr<IWheelTickPerceptor> Ptr;
	typedef boost::shared_ptr<const IWheelTickPerceptor> ConstPtr;

	virtual ~IWheelTickPerceptor(){};

	/** Retrieve the ticks of the perceptor.
	 *
	 * \returns The perceptor ticks.
	 */
	virtual const int &getTicks() const = 0;

	/** Retrieve the direction of the perceptor.
	 *
	 * \returns The perceptor direction.
	 */
	virtual const char &getDirection() const = 0;
};
}
