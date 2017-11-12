#pragma once

#include <string>

namespace taco
{
class Maneuver
{
  public:
	Maneuver(const std::string &driveInstruction, const int16_t &sector, const int16_t &maneuverID)
	{
		_driveInstruction = driveInstruction;
		_sector = sector;
		_maneuverID = maneuverID;
	};
	virtual ~Maneuver(){};

	virtual const std::string &getDriveInstruction() const
	{
		return _driveInstruction;
	}

	virtual const std::int16_t &getSector() const
	{
		return _sector;
	}

	virtual const std::int16_t &getManeuverId() const
	{
		return _maneuverID;
	}

  protected:
	std::string _driveInstruction;
	std::int16_t _sector;
	std::int16_t _maneuverID;
};
}
