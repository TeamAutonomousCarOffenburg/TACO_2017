#pragma once

#include "IEffector.h"
#include "IManeuverStatusEffector.h"
#include "IPositionEffector.h"
#include "IValueEffector.h"

#include <boost/smart_ptr.hpp>
#include <vector>

namespace taco
{
/**
 * General Interface for the action component.
 * The IAction is responsible for buffering the actions of actuators for transmission.
 *
 * \author Stefan Glaser
 */
class IAction
{
  public:
	typedef boost::shared_ptr<IAction> Ptr;
	typedef boost::shared_ptr<const IAction> ConstPtr;

	virtual ~IAction(){};

	virtual IEffector::Ptr getEffector(const std::string &name) = 0;
	virtual IEffector::ConstPtr getEffector(const std::string &name) const = 0;

	virtual IDoubleValueEffector::Ptr getServoDriveEffector(const std::string &name) = 0;
	virtual IDoubleValueEffector::ConstPtr getServoDriveEffector(const std::string &name) const = 0;

	virtual IDoubleValueEffector::Ptr getMotorEffector(const std::string &name) = 0;
	virtual IDoubleValueEffector::ConstPtr getMotorEffector(const std::string &name) const = 0;

	virtual IBoolValueEffector::Ptr getLightEffector(const std::string &name) = 0;
	virtual IBoolValueEffector::ConstPtr getLightEffector(const std::string &name) const = 0;

	virtual IManeuverStatusEffector::ConstPtr getManeuverStatusEffector(const std::string &name) const = 0;
	virtual IManeuverStatusEffector::Ptr getManeuverStatusEffector(const std::string &name) = 0;

	virtual IPositionEffector::ConstPtr getPositionEffector(const std::string &name) const = 0;
	virtual IPositionEffector::Ptr getPositionEffector(const std::string &name) = 0;
};
}
