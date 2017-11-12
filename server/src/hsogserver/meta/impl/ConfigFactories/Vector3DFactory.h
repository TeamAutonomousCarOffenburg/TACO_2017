#pragma once

#include "ConfigFactory.h"
#include <eigen3/Eigen/Dense>
#include <string>

using namespace std;

// Hilfsklasse
// Erstellt ein Vector3d-Objekt
class Vector3DFactory : public ConfigFactory<typename Eigen::Vector3d>
{
	double x;
	double y;
	double z;

  public:
	virtual ~Vector3DFactory(){};

	virtual Eigen::Vector3d getConfig()
	{
		return Eigen::Vector3d(x, y, z);
	};

	virtual bool Double(std::vector<std::string> &context, double value)
	{
		if (context.back().compare("x") == 0) {
			x = value;
			return true;
		}
		if (context.back().compare("y") == 0) {
			y = value;
			return true;
		}
		if (context.back().compare("z") == 0) {
			z = value;
			return true;
		}
		return false;
	};
};
