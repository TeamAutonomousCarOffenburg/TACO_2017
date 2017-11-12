#pragma once

#include "utils/geometry/Polygon.h"
#include "utils/geometry/Pose2D.h"
#include "utils/geometry/Pose3D.h"

class EncoderUtilities
{
  public:
	static void encodePose2D(Writer<StringBuffer> *writer, taco::Pose2D pose)
	{
		writer->StartObject();
		writer->Key("x");
		writer->Double(pose.x());
		writer->Key("y");
		writer->Double(pose.y());
		writer->Key("angle");
		writer->StartObject();
		writer->Key("angle");
		writer->Double(pose.getAngle().rad());
		writer->EndObject();
		writer->EndObject();
	}

	static void encodeVector3d(Writer<StringBuffer> *writer, Eigen::Vector3d vec)
	{
		writer->StartObject();
		writer->Key("x");
		writer->Double(vec.x());
		writer->Key("y");
		writer->Double(vec.y());
		writer->Key("z");
		writer->Double(vec.z());
		writer->EndObject();
	}

	static void encodeVector2d(Writer<StringBuffer> *writer, Eigen::Vector2d vec)
	{
		writer->StartObject();
		writer->Key("x");
		writer->Double(vec.x());
		writer->Key("y");
		writer->Double(vec.y());
		writer->EndObject();
	}

	static void encodePolygon(Writer<StringBuffer> *writer, taco::Polygon::ConstPtr poly)
	{
		writer->StartObject();
		writer->Key("centroid");
		encodeVector2d(writer, poly->getCentroid());
		writer->Key("area");
		writer->Double(poly->getArea());
		writer->Key("points");
		writer->StartArray();
		for (auto it = poly->getPoints().begin(); it != poly->getPoints().end(); it++) {
			encodeVector2d(writer, *it);
		}
		writer->EndArray();
		writer->EndObject();
	}
};
