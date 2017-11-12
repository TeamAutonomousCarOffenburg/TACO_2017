#pragma once

#include "action/IAction.h"
#include "meta/ICarMetaModel.h"

#include <map>

namespace taco
{
/**
 * The Action manages the effectors of the system.
 *
 * \author Stefan Glaser
 */
class Action : public virtual IAction
{
  public:
	typedef boost::shared_ptr<Action> Ptr;
	typedef boost::shared_ptr<const Action> ConstPtr;

	Action(ICarMetaModel::ConstPtr carMetaModel);
	virtual ~Action();

	virtual IEffector::Ptr getEffector(const std::string &name);
	virtual IEffector::ConstPtr getEffector(const std::string &name) const;

	virtual IDoubleValueEffector::Ptr getServoDriveEffector(const std::string &name);
	virtual IDoubleValueEffector::ConstPtr getServoDriveEffector(const std::string &name) const;

	virtual IDoubleValueEffector::Ptr getMotorEffector(const std::string &name);
	virtual IDoubleValueEffector::ConstPtr getMotorEffector(const std::string &name) const;

	virtual IBoolValueEffector::Ptr getLightEffector(const std::string &name);
	virtual IBoolValueEffector::ConstPtr getLightEffector(const std::string &name) const;

	virtual IManeuverStatusEffector::Ptr getManeuverStatusEffector(const std::string &name);
	virtual IManeuverStatusEffector::ConstPtr getManeuverStatusEffector(const std::string &name) const;

	virtual IPositionEffector::Ptr getPositionEffector(const std::string &name);
	virtual IPositionEffector::ConstPtr getPositionEffector(const std::string &name) const;

  protected:
	/** The map of effectors. */
	std::map<std::string, IEffector::Ptr> _effectorMap;
};
}
