#include "RoadSign.h"

using namespace taco;

RoadSign::RoadSign(const SignType &type, const Pose2D &pose) : VisibleObject(getNameForSign(type), pose), _sign(type)
{
}

RoadSign::~RoadSign()
{
}

const SignType &RoadSign::getSignType() const
{
	return _sign;
}

void RoadSign::update(const Pose2D &pose)
{
	_pose = Pose2D::average(_pose, pose, _seenCount, 1);
	_seenCount++;
}

int RoadSign::getSeenCount() const
{
	return _seenCount;
}

const std::string RoadSign::getNameForSign(const SignType &type)
{
	switch (type) {
	case taco::UnmarkedIntersection:
		return "UnmarkedIntersection";
	case taco::Stop:
		return "Stop";
	case taco::ParkingArea:
		return "ParkingArea";
	case taco::HaveWay:
		return "HaveWay";
	case taco::AheadOnly:
		return "AheadOnly";
	case taco::GiveWay:
		return "GiveWay";
	case taco::Crosswalk:
		return "Crosswalk";
	case taco::Roundabout:
		return "Roundabout";
	case taco::NoOvertaking:
		return "NoOvertaking";
	case taco::NoEntryVehicularTraffic:
		return "NoEntryVehicularTraffic";
	case taco::TestCourseA9:
		return "TestCourseA9";
	case taco::OneWayStreet:
		return "OneWayStreet";
	case taco::RoadWorks:
		return "RoadWorks";
	case taco::KMH_50:
		return "KMH_50";
	case taco::KMH_100:
		return "KMH_100";
	default:
		return "NoMatch";
	}
}
