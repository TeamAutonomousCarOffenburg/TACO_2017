#include "ConfigurationReader.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <iostream>
#include <utils/geometry/Angle.h>
#include <zconf.h>

using namespace taco;
using namespace boost;
using namespace boost::property_tree;

void ConfigurationReader::loadFromFile(const std::string &filename, EnvironmentConfiguration &config)
{
	ptree tree;

	try {
		read_xml(filename, tree);
		load(tree, config);
	} catch (...) {
		std::cerr << "Could not read config file '" << filename << "'." << std::endl;
	}
}

void ConfigurationReader::loadFromString(const std::string &xml_string, EnvironmentConfiguration &config)
{
	std::stringstream ss(xml_string);

	ptree tree;

	try {
		read_xml(ss, tree);
		load(tree, config);
	} catch (...) {
		std::cerr << "Could not read config file." << std::endl;
	}
}

void ConfigurationReader::load(ptree tree, EnvironmentConfiguration &config)
{
	std::vector<ConfigRoadSign> roadSigns = config.getRoadSigns();
	std::vector<ParkingSpace> parkingSpaces = config.getParkingSpaces();
	for (const ptree::value_type &configInfo : tree.get_child("configuration")) {
		if (configInfo.first == "roadSign") {
			int16_t id = configInfo.second.get<int16_t>("<xmlattr>.id", 0);
			float_t xPos = configInfo.second.get<float>("<xmlattr>.x", 0);
			float_t yPos = configInfo.second.get<float>("<xmlattr>.y", 0);
			float_t radius = configInfo.second.get<float>("<xmlattr>.radius", 0);
			int16_t direction = configInfo.second.get<int16_t>("<xmlattr>.direction", 90);

			roadSigns.push_back(ConfigRoadSign(id, xPos, yPos, radius, direction));
		} else if (configInfo.first == "parkingSpace") {
			int16_t id = configInfo.second.get<int16_t>("<xmlattr>.id", 0);
			float_t xPos = configInfo.second.get<float>("<xmlattr>.x", 0);
			float_t yPos = configInfo.second.get<float>("<xmlattr>.y", 0);
			int8_t state = configInfo.second.get<int8_t>("<xmlattr>.status", 0);
			int16_t direction = configInfo.second.get<int16_t>("<xmlattr>.direction", 90);

			parkingSpaces.push_back(ParkingSpace(id, xPos, yPos, state, direction));
		}
	}

	config.setRoadSigns(roadSigns);
	config.setParkingSpaces(parkingSpaces);
}
