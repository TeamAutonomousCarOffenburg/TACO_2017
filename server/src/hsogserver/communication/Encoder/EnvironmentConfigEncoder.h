#pragma once

#include "EncoderUtilities.h"
#include "IEncoder.h"
#include <string>
#include <utils/configurationreader/ConfigurationReader.h>
#include <utils/configurationreader/EnvironmentConfiguration.h>

class EnvironmentConfigEncoder : public IEncoder
{
  public:
	EnvironmentConfigEncoder(taco::EnvironmentConfiguration::Ptr environmentConfig)
	{
		std::string pathToFile = "/home/aadc/AADC/configuration_files/roadSigns.xml";

		// Fetch the list of roadsigns and parkingspaces
		std::vector<ConfigRoadSign> roadSigns;
		std::vector<ParkingSpace> parkingSpaces;
		config = EnvironmentConfiguration(roadSigns, parkingSpaces);
		ConfigurationReader::loadFromFile(pathToFile, config);
	}

	virtual ~EnvironmentConfigEncoder(){};

	virtual void encode(Writer<StringBuffer> *writer)
	{
		std::vector<ConfigRoadSign> roadSigns = config.getRoadSigns();

		writer->Key(taco::AADCCar::ENVIRONMENT_CONFIGURATION.c_str());
		writer->StartObject();

		writer->Key("roadSigns");
		writer->StartArray();
		for (ConfigRoadSign &roadSign : roadSigns) {
			writer->StartObject();
			writer->Key("sign");
			writer->Int(roadSign.getId());
			writer->Key("pose");
			EncoderUtilities::encodePose2D(
					writer, Pose2D(roadSign.getXPos(), roadSign.getYPos(), taco::Angle::deg(roadSign.getDirection())));
			writer->EndObject();
		}
		writer->EndArray();

		std::vector<ParkingSpace> parkingSpaces = config.getParkingSpaces();

		writer->Key("parkingSpaces");
		writer->StartArray();
		for (ParkingSpace &parkingSpace : parkingSpaces) {
			writer->StartObject();
			writer->Key("id");
			writer->Int(parkingSpace.getId());
			writer->Key("pose");
			EncoderUtilities::encodePose2D(writer, Pose2D(parkingSpace.getXPos(), parkingSpace.getYPos(),
														   taco::Angle::deg(parkingSpace.getDirection())));
			writer->Key("state");
			writer->Int(parkingSpace.getState());
			writer->EndObject();
		}
		writer->EndArray();
		writer->EndObject();
	}

  private:
	taco::EnvironmentConfiguration config;
};
