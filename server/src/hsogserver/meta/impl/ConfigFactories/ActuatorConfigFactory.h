#pragma once

#include "../../IActuatorConfig.h"
#include "../ActuatorConfig.h"
#include "ConfigFactory.h"
#include <string>
// Factory für eine ActuatorConfig.
// Implementiert die "Wertefunktionen" für String. Int und Double werden nicht benötigt und damit nicht überschrieben.
// Kindklassen können die "Wertefunktionen" weiterhin überschreiben wenn weitere Werte hinzukommen. (Z.B. bei einerm
// ServoDriveActuator) Kindklassen rufen weiterhin die Funktionen der Elternklasse auf wenn sie mit dem übergebenen
// context nichts anfangen können. Die Elternklasse (ActuatorConfigFactory) kümmert sich dann um den Wert oder gibt
// false zurück falls ein ungültiger context übergeben wurde.
class ActuatorConfigFactory : public ConfigFactory<taco::IActuatorConfig::ConstPtr>
{
  protected:
	std::string name;

  public:
	virtual ~ActuatorConfigFactory(){};

	virtual taco::IActuatorConfig::ConstPtr getConfig()
	{
		return boost::make_shared<const taco::ActuatorConfig>(name);
	}

	virtual bool String(std::vector<std::string> &context, const char *value)
	{
		if (context.back().compare("name") == 0) {
			name = value;
			return true;
		}
		return false;
	};
};
