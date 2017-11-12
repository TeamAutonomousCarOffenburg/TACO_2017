#pragma once

#include "IConfigFactory.h"

// Abstrakte Klasse.
// Bietet Standardimplementation für die einzelnen "Wertfunktionen".
// Funktionen die von einer Kindklasse verwendet werden, werden von dieser Überschrieben.
// Die anderen Funktionen sollten nicht verwendet werden und geben deshalb immer false zurück.
// Dadurch müssen nicht verwendete Funktionen in den Kindklassen nicht implementiert werden.
template <typename CONFIG> class ConfigFactory : public IConfigFactory
{
  public:
	virtual ~ConfigFactory()
	{
	}

	virtual bool String(std::vector<std::string> &context, const char *value)
	{
		return false;
	}

	virtual bool Double(std::vector<std::string> &context, double value)
	{
		return false;
	}

	virtual bool Int(std::vector<std::string> &context, int value)
	{
		return false;
	}

	// Gibt die Config zurück welche von der Factory erstellt wird. Typ wird in der Kindklasse als Templateparameter
	// angegeben.
	virtual CONFIG getConfig() = 0;
};
