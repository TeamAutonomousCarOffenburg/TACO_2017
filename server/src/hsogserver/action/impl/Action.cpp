#include "Action.h"

#include "ManeuverStatusEffector.h"
#include "ObstacleEffector.h"
#include "ParkingSpaceEffector.h"
#include "PositionEffector.h"
#include "RoadSignEffector.h"
#include "ValueEffector.h"

using namespace taco;

Action::Action(ICarMetaModel::ConstPtr carMetaModel)
{
	// Add Effectors
	// Add ServoDrive effectors
	std::vector<IServoDriveConfig::ConstPtr> servoDriveConfigs = carMetaModel->getServoDriveConfigs();
	for (unsigned int i = 0; i < servoDriveConfigs.size(); i++) {
		IDoubleValueEffector::Ptr effector(
				boost::make_shared<DoubleValueEffector>(servoDriveConfigs[i]->getEffectorName()));
		_effectorMap.insert(make_pair(effector->getName(), effector));
	}

	// Add Motor actuators
	std::vector<IActuatorConfig::ConstPtr> effectorConfigs = carMetaModel->getMotorConfigs();
	for (unsigned int i = 0; i < effectorConfigs.size(); i++) {
		IDoubleValueEffector::Ptr effector(
				boost::make_shared<DoubleValueEffector>(effectorConfigs[i]->getEffectorName()));
		_effectorMap.insert(make_pair(effector->getName(), effector));
	}

	// Add Light actuators
	effectorConfigs = carMetaModel->getLightConfigs();
	for (unsigned int i = 0; i < effectorConfigs.size(); i++) {
		IBoolValueEffector::Ptr effector(boost::make_shared<BoolValueEffector>(effectorConfigs[i]->getEffectorName()));
		_effectorMap.insert(make_pair(effector->getName(), effector));
	}

	effectorConfigs = carMetaModel->getManeuverStatusConfigs();
	for (unsigned int i = 0; i < effectorConfigs.size(); i++) {
		IManeuverStatusEffector::Ptr effector(
				boost::make_shared<ManeuverStatusEffector>(effectorConfigs[i]->getEffectorName()));
		_effectorMap.insert(make_pair(effector->getName(), effector));
	}

	IPositionEffector::Ptr posEffector(boost::make_shared<PositionEffector>("Position"));
	_effectorMap.insert(make_pair(posEffector->getName(), posEffector));

	IRoadSignEffector::Ptr signEffector(boost::make_shared<RoadSignEffector>("RoadSign"));
	_effectorMap.insert(make_pair(signEffector->getName(), signEffector));

	IParkingSpaceEffector::Ptr parkingspaceEffector(boost::make_shared<ParkingSpaceEffector>("ParkingSpace"));
	_effectorMap.insert(make_pair(parkingspaceEffector->getName(), parkingspaceEffector));

	IObstacleEffector::Ptr obstacleEffector(boost::make_shared<ObstacleEffector>("Obstacle"));
	_effectorMap.insert(make_pair(obstacleEffector->getName(), obstacleEffector));
}

Action::~Action()
{
}

IEffector::ConstPtr Action::getEffector(const std::string &name) const
{
	IEffector::ConstPtr servoEffector;

	auto it = _effectorMap.find(name);
	if (it != _effectorMap.end()) {
		servoEffector = boost::dynamic_pointer_cast<const IEffector>(it->second);
	}

	return servoEffector;
}

IEffector::Ptr Action::getEffector(const std::string &name)
{
	IEffector::Ptr servoEffector;

	auto it = _effectorMap.find(name);
	if (it != _effectorMap.end()) {
		servoEffector = it->second;
	}

	return servoEffector;
}

IDoubleValueEffector::ConstPtr Action::getServoDriveEffector(const std::string &name) const
{
	IDoubleValueEffector::ConstPtr servoEffector;

	auto it = _effectorMap.find(name);
	if (it != _effectorMap.end()) {
		servoEffector = boost::dynamic_pointer_cast<const IDoubleValueEffector>(it->second);
	}

	return servoEffector;
}

IDoubleValueEffector::Ptr Action::getServoDriveEffector(const std::string &name)
{
	IDoubleValueEffector::Ptr servoEffector;

	auto it = _effectorMap.find(name);
	if (it != _effectorMap.end()) {
		servoEffector = boost::dynamic_pointer_cast<IDoubleValueEffector>(it->second);
	}

	return servoEffector;
}

IDoubleValueEffector::ConstPtr Action::getMotorEffector(const std::string &name) const
{
	IDoubleValueEffector::ConstPtr motorEffector;

	auto it = _effectorMap.find(name);
	if (it != _effectorMap.end()) {
		motorEffector = boost::dynamic_pointer_cast<const IDoubleValueEffector>(it->second);
	}

	return motorEffector;
}

IDoubleValueEffector::Ptr Action::getMotorEffector(const std::string &name)
{
	IDoubleValueEffector::Ptr motorEffector;

	auto it = _effectorMap.find(name);
	if (it != _effectorMap.end()) {
		motorEffector = boost::dynamic_pointer_cast<IDoubleValueEffector>(it->second);
	}

	return motorEffector;
}

IBoolValueEffector::ConstPtr Action::getLightEffector(const std::string &name) const
{
	IBoolValueEffector::ConstPtr lightEffector;

	auto it = _effectorMap.find(name);
	if (it != _effectorMap.end()) {
		lightEffector = boost::dynamic_pointer_cast<const IBoolValueEffector>(it->second);
	}

	return lightEffector;
}

IBoolValueEffector::Ptr Action::getLightEffector(const std::string &name)
{
	IBoolValueEffector::Ptr lightEffector;

	auto it = _effectorMap.find(name);
	if (it != _effectorMap.end()) {
		lightEffector = boost::dynamic_pointer_cast<IBoolValueEffector>(it->second);
	}

	return lightEffector;
}

IManeuverStatusEffector::ConstPtr Action::getManeuverStatusEffector(const std::string &name) const
{
	IManeuverStatusEffector::ConstPtr statusEffector;

	auto it = _effectorMap.find(name);
	if (it != _effectorMap.end()) {
		statusEffector = boost::dynamic_pointer_cast<IManeuverStatusEffector>(it->second);
	}

	return statusEffector;
}

IManeuverStatusEffector::Ptr Action::getManeuverStatusEffector(const std::string &name)
{
	IManeuverStatusEffector::Ptr ManeuverStatusEffector;

	auto it = _effectorMap.find(name);
	if (it != _effectorMap.end()) {
		ManeuverStatusEffector = boost::dynamic_pointer_cast<IManeuverStatusEffector>(it->second);
	}

	return ManeuverStatusEffector;
}

IPositionEffector::ConstPtr Action::getPositionEffector(const std::string &name) const
{
	IPositionEffector::ConstPtr positionEffector;

	auto it = _effectorMap.find(name);
	if (it != _effectorMap.end()) {
		positionEffector = boost::dynamic_pointer_cast<IPositionEffector>(it->second);
	}

	return positionEffector;
}

IPositionEffector::Ptr Action::getPositionEffector(const std::string &name)
{
	IPositionEffector::Ptr PositionEffector;

	auto it = _effectorMap.find(name);
	if (it != _effectorMap.end()) {
		PositionEffector = boost::dynamic_pointer_cast<IPositionEffector>(it->second);
	}

	return PositionEffector;
}
