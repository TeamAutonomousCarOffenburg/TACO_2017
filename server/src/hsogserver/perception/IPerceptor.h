#pragma once

#include <boost/smart_ptr.hpp>
#include <string>

namespace taco
{
/**
 * Interface for all known perceptors of the system.
 *
 * \author Stefan Glaser
 */
class IPerceptor
{
  public:
	typedef boost::shared_ptr<IPerceptor> Ptr;
	typedef boost::shared_ptr<const IPerceptor> ConstPtr;

	virtual ~IPerceptor(){};

	/** Retrieve the name of the perceptor.
	 *
	 * \returns The perceptor name.
	 */
	virtual const std::string &getName() const = 0;

	/** Retrieve the time of the measurement (in microseconds).
	 *
	 * \returns The measurement time (ys)(10e-6s).
	 */
	virtual const long &getTime() const = 0;
};
}
