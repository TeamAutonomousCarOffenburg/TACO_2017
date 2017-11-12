#pragma once

#include "ConfigFactory.h"
#include "Vector3DFactory.h"

// Hilfsklasse
// Erstellt ein AngleAxisd-Objekt
class AngleAxisdFactory : public ConfigFactory<Eigen::AngleAxisd>
{
	double angle;
	Vector3DFactory vec;

  public:
	virtual ~AngleAxisdFactory(){};

	virtual Eigen::AngleAxisd getConfig()
	{
		return Eigen::AngleAxisd(angle, vec.getConfig());
	};

	virtual bool Double(std::vector<std::string> &context, double value)
	{
		if (context.back().compare("angle") == 0) {
			angle = value * M_PI / 180.0;
			return true;
		}
		// Default: Wenn nicht angle kann es nur noch x,y oder z sein. Die Prüfung darauf übernimmt die
		// VectorFactory. Diese gibt false zurück falls es keines der genannten ist.
		return vec.Double(context, value);
	};
};
