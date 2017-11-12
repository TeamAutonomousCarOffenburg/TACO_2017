#pragma once

#include <boost/property_tree/ptree.hpp>
#include <perception/impl/Maneuver.h>
#include <string>
#include <vector>

namespace taco
{
class DriveInstructionReader
{
  public:
	static void loadFromFile(const std::string &filename, std::vector<Maneuver> &driveInstructions);
	static void loadFromString(const std::string &xml_string, std::vector<Maneuver> &driveInstructions);

  private:
	static void load(boost::property_tree::ptree tree, std::vector<Maneuver> &driveInstructions);
};
}
