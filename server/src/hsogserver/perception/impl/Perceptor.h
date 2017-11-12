#pragma once

#include "perception/IPerceptor.h"

#include <string>

namespace taco
{
/**
 * The Perceptor class acs as base class for all perceptors.
 *
 * \author Stefan Glaser
 */
class Perceptor : public virtual IPerceptor
{
  public:
	typedef boost::shared_ptr<Perceptor> Ptr;
	typedef boost::shared_ptr<const Perceptor> ConstPtr;

	Perceptor(const std::string &name, const long &time) : _name(name), _time(time){};
	virtual ~Perceptor(){};

	virtual const std::string &getName() const
	{
		return _name;
	};

	virtual const long &getTime() const
	{
		return _time;
	};

  protected:
	/** The name of the perceptor. */
	std::string _name;

	/** The timestamp of the perception. */
	long _time;
};
}
