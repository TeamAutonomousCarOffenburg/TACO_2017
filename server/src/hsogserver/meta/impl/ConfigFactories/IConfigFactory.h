#pragma once

#include <string>
#include <vector>

// Interface für die Factories. Werte werden mit einem Kontext übergeben
// Der Kontext hierbei ist eine Liste alle Schlüssel der darüberliegenden Objekte
class IConfigFactory
{
  public:
	virtual ~IConfigFactory()
	{
	}

	virtual bool String(std::vector<std::string> &context, const char *value) = 0;
	virtual bool Double(std::vector<std::string> &context, double value) = 0;
	virtual bool Int(std::vector<std::string> &context, int value) = 0;
};
