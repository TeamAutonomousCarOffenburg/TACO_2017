#pragma once

#include "EnvironmentConfiguration.h"
#include <boost/property_tree/ptree.hpp>
#include <string>
#include <vector>

namespace taco
{
class ConfigurationReader
{
  public:
	static void loadFromFile(const std::string &filename, EnvironmentConfiguration &config);
	static void loadFromString(const std::string &xml_string, EnvironmentConfiguration &config);

  private:
	static void load(boost::property_tree::ptree tree, EnvironmentConfiguration &config);
};
}