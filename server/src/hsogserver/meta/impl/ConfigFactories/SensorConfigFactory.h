#pragma once

#include "../../ISensorConfig.h"
#include "../SensorConfig.h"
#include "AngleAxisdFactory.h"
#include "ConfigFactory.h"
#include "Vector3DFactory.h"

// Factory für eine SensorConfig.
// Implementiert die "Wertefunktionen" für String und Double. Int wird nicht benötigt und damit nicht überschrieben
// Kindklassen können die "Wertefunktionen" weiterhin überschreiben wenn weitere Werte hinzukommen. (Z.B. bei einerm
// DistanceSensor) Kindklassen rufen weiterhin die Funktionen der Elternklasse auf wenn sie mit dem übergebenen context
// nichts anfangen können. Die Elternklasse (SensorConfigFactory) kümmert sich dann um den Wert oder gibt false zurück
// falls ein ungültiger context übergeben wurde.
class SensorConfigFactory : public ConfigFactory<taco::ISensorConfig::ConstPtr>
{
  protected:
	std::string name;
	Vector3DFactory vec;
	AngleAxisdFactory ang;

  public:
	virtual ~SensorConfigFactory(){};

	virtual taco::ISensorConfig::ConstPtr getConfig()
	{
		return boost::make_shared<const taco::SensorConfig>(name, vec.getConfig(), ang.getConfig());
	};

	virtual bool String(std::vector<std::string> &context, const char *value)
	{
		if (context.back().compare("name") == 0) {
			name = value;
			return true;
		}
		return false;
	};

	virtual bool Double(std::vector<std::string> &context, double value)
	{
		// wenn position oder axis wird es an jeweilige Factory weitergegeben. Ob es sich um x,y oder z handelt wird
		// dort geprüft.
		if (context[context.size() - 2].compare("position") == 0) {
			return vec.Double(context, value);
		}
		if (string(context[context.size() - 2]).compare("axis") == 0) {
			return ang.Double(context, value);
		}
		// Default: Wenn keine der obigen kann es nur noch angle sein. Ist es das nicht gibt ang.Double false zurück
		// Somit wird nicht doppelt auf den String angle geprüft.
		return ang.Double(context, value);
	}
};
