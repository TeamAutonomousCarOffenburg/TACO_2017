#pragma once

#include <iostream>

class IHandler
{
  public:
	// Abstrakte Funktion. name gibt den Key des JSON-Attributes an. Value ist ein Pointer auf den Wert
	// Hinter dem Value-Pointer könne sich je nach Attribut verschiedene Werte verbergen. Die Implementierung
	virtual bool handle(std::string name, void *value) = 0;
	// Name des JSON-Attributs für welches der Handler verwendet werden soll
	virtual std::string getName() = 0;
	virtual ~IHandler()
	{
	}
};
