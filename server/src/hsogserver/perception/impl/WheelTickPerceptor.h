#pragma once

#include "Perceptor.h"
#include "perception/IWheelTickPerceptor.h"

#include <string>

namespace taco
{
/**
 * The WheelTickPerceptor class represents a wheel tick perception.
 *
 * \author Stefan Glaser
 */
class WheelTickPerceptor : public Perceptor, public virtual IWheelTickPerceptor
{
  public:
	typedef boost::shared_ptr<WheelTickPerceptor> Ptr;
	typedef boost::shared_ptr<const WheelTickPerceptor> ConstPtr;

	WheelTickPerceptor(const std::string &name, const long &time, const int &ticks, const char &dir)
		: Perceptor(name, time), _ticks(ticks), _dir(dir){};
	virtual ~WheelTickPerceptor(){};

	virtual const int &getTicks() const
	{
		return _ticks;
	};

	virtual const char &getDirection() const
	{
		return _dir;
	};

  protected:
	/** The ticks. */
	int _ticks;

	/** The direction of rotation. */
	char _dir;
};
}
