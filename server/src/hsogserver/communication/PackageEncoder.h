#pragma once

#include "../perception/IPerception.h"
#include "Encoder/IEncoder.h"
#include "detection/FloorNormalDetection.h"
#include "detection/ILaneDetection.h"
#include "detection/IObjectDetection.h"
#include <chrono>
#include <meta/ICarMetaModel.h>
#include <utils/configurationreader/EnvironmentConfiguration.h>

using namespace rapidjson;

// Codiert das Paket zum versenden an den Client
// Nutzt einzelne Encoder für diese Aufgabe
class PackageEncoder
{
  public:
	// Konstruktor. Erstellt die einzelnen Encoder-Objekte
	// In der Implementierung werden hier weitere Encoder eingefügt wenn weitere Daten übertragen werden sollen.
	PackageEncoder(taco::IPerception::Ptr p, taco::ICarMetaModel::Ptr cm, taco::ILaneDetection::Ptr laneDetection,
			taco::IObjectDetection::Ptr objectDetection, taco::FloorNormalDetection &floorNormal,
			taco::EnvironmentConfiguration::Ptr &environmentConfig);
	~PackageEncoder();
	// Erstellt den JSON-String zur Versendung und schreibt diesen in buf.
	size_t encode(unsigned char *buf, std::chrono::system_clock::time_point lastClientConnectionTime);

  private:
	std::vector<IEncoder *> Encoderlist;
};
