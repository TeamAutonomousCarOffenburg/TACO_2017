#pragma once

#include "IJuryConstants.h"
#include "IPerceptor.h"

namespace taco
{
class IJuryPerceptor : public virtual IPerceptor
{
  public:
	typedef boost::shared_ptr<IJuryPerceptor> Ptr;
	typedef boost::shared_ptr<const IJuryPerceptor> ConstPtr;

	virtual ~IJuryPerceptor(){};

	virtual const JuryAction getAction() const = 0;
	virtual const int getManeuverId() const = 0;
};
}
