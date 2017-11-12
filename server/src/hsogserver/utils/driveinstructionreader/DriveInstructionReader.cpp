#include "DriveInstructionReader.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <iostream>

using namespace taco;
using namespace boost;
using namespace boost::property_tree;

void DriveInstructionReader::loadFromFile(const std::string &filename, std::vector<Maneuver> &driveInstructions)
{
	ptree tree;

	try {
		read_xml(filename, tree);
		load(tree, driveInstructions);
	} catch (...) {
		std::cerr << "Could not read maneuver list." << std::endl;
	}
}

void DriveInstructionReader::loadFromString(const std::string &xml_string, std::vector<Maneuver> &driveInstructions)
{
	std::stringstream ss(xml_string);

	ptree tree;

	try {
		read_xml(ss, tree);
		load(tree, driveInstructions);
	} catch (...) {
		std::cerr << "Could not read maneuver list." << std::endl;
	}
}

void DriveInstructionReader::load(ptree tree, std::vector<Maneuver> &driveInstructions)
{
	// Clear maybe present instructions
	driveInstructions.clear();

	for (const ptree::value_type &v : tree.get_child("AADC-Maneuver-List")) {
		if (v.first == "AADC-Sector") {
			int16_t sector = v.second.get<int>("<xmlattr>.id");
			for (const ptree::value_type &maneuver : v.second) {
				if (maneuver.first == "AADC-Maneuver") {
					int id = maneuver.second.get<int>("<xmlattr>.id", -1);
					if (id != driveInstructions.size()) {
						std::cerr << "OOOPS!!! Maneuver ID in maneuver-xml file NOT strictly increasing by one!!!"
								  << std::endl;
					}
					std::string instruction = maneuver.second.get<std::string>("<xmlattr>.action");
					driveInstructions.push_back(Maneuver(instruction, sector, id));
				}
			}
		}
	}
}
