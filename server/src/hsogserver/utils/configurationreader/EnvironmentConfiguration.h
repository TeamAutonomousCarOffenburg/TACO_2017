#pragma once

#include "ConfigRoadSign.h"
#include "ParkingSpace.h"
#include <boost/smart_ptr.hpp>
#include <iostream>
#include <vector>

using namespace taco;
namespace taco
{
class EnvironmentConfiguration
{
  public:
	EnvironmentConfiguration(
			const std::vector<ConfigRoadSign> &roadSigns, const std::vector<ParkingSpace> &parkingSpaces)
	{
		_roadSigns = roadSigns;
		_parkingSpaces = parkingSpaces;
	};

	virtual ~EnvironmentConfiguration(){};

	EnvironmentConfiguration(){};

	virtual const std::vector<ConfigRoadSign> &getRoadSigns() const
	{
		return _roadSigns;
	}

	virtual const std::vector<ParkingSpace> &getParkingSpaces() const
	{
		return _parkingSpaces;
	}

	virtual void setRoadSigns(const std::vector<ConfigRoadSign> &roadSigns)
	{
		_roadSigns = roadSigns;
	}

	virtual void setParkingSpaces(const std::vector<ParkingSpace> &parkingSpaces)
	{
		_parkingSpaces = parkingSpaces;
	}

	typedef boost::shared_ptr<EnvironmentConfiguration> Ptr;

  protected:
	std::vector<ConfigRoadSign> _roadSigns;
	std::vector<ParkingSpace> _parkingSpaces;
};
}