#pragma once

#include <boost/shared_ptr.hpp>
#include <cstdint>
#include <map>
#include <string>

#include "perception/IPerceptor.h"
#include "utils/geometry/Pose2D.h"

namespace taco
{
class IEventLogger
{
  public:
	typedef boost::shared_ptr<IEventLogger> Ptr;
	typedef boost::shared_ptr<const IEventLogger> ConstPtr;

	virtual void logDecodeEvent(uint32_t timestamp, std::string name, double value) const = 0;
	virtual void logPose(const std::string &name, const taco::Pose2D &pose) const = 0;
	virtual void logTimeTaken(const std::string &name, long duration) const = 0;

	virtual void stop() const = 0;
	virtual void start() const = 0;

	virtual ~IEventLogger(){};
};
}
