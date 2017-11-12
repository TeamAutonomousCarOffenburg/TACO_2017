#pragma once

#include "../action/IAction.h"
#include "Handler/IHandler.h"
#include <meta/ICarMetaModel.h>

class PackageDecoder
{
  public:
	// Konstruktor. Erstellt die einzelnen Handler-Objekte
	// In der Implementierung werden hier weitere Handler eingefügt wenn weitere Daten übertragen werden sollen.
	PackageDecoder(taco::IAction::Ptr ap, taco::ICarMetaModel::Ptr cm);
	~PackageDecoder();
	void decode(char *msg);

  private:
	std::vector<IHandler *> Handlerlist;
	taco::IAction::Ptr ap;
};
