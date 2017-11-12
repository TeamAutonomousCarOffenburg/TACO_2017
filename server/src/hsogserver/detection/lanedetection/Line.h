#pragma once

#include <opencv2/core/core.hpp>
#include <utils/geometry/Angle.h>

namespace taco
{
class Line
{
  public:
	Line();
	Line(cv::Point &s, cv::Point &e);
	Line(int x1, int y1, int x2, int y2);

	virtual ~Line();

	void swap(int *a, int *b)
	{
		int t = *a;
		*a = *b;
		*b = t;
	}

	/**
	 * returns a vector of points that are pixel points on this line
	 */
	void getLinePoints(std::vector<cv::Point> &points);

	/**
	 * returns 0 if this line has the same slope as the line passed by the coordinates (with passed +-precision)
	 * -1 if
	 */
	int checkSameDirection(int x1, int y1, int x2, int y2, const Angle &precision);
	int checkSameDirection(Line &line2, const Angle &precision);

	/**
	 * returns the slope of this line  (range -pi to pi inclusive)
	 */
	Angle getSlope();

	/**
	 * returns the length of the line
	 */
	float getLength();

	/**
	 * For now we consider a line (0,0,0,0) invalid
	 */
	bool isValid();

	cv::Point start;
	cv::Point end;
};
}
