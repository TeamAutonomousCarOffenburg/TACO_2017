#pragma once

#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

using namespace rapidjson;
// Basisinterface für alle Encoder
class IEncoder
{
  public:
	// Abstrakte Funktion. Mifhilfe des Writes wird hier ein Teil des JSON-Objekts geschrieben
	// Dies kann ein einfacher Key-Value-Wert sein, aber auch ein ganzes Array oder Objekt
	// je nachdem was hier Codiert wird.
	virtual void encode(Writer<StringBuffer> *writer) = 0;
	virtual ~IEncoder()
	{
	}
};
