#include "Line.h"

using namespace taco;
using namespace cv;
using namespace std;

Line::Line()
{
}

Line::Line(Point &s, Point &e)
{
	start = s;
	end = e;
}

Line::Line(int x1, int y1, int x2, int y2)
{
	start = Point(x1, y1);
	end = Point(x2, y2);
}

Line::~Line()
{
}

/**
 * Find points on scanline using midpoint line algorithm
 */
void Line::getLinePoints(vector<Point> &points)
{
	int x1 = start.x;
	int y1 = start.y;
	int x2 = end.x;
	int y2 = end.y;
	int dx, dy, d, incry, incre, incrne;
	bool slopegt1 = false;
	bool reverse = false;
	dx = abs(x1 - x2);
	dy = abs(y1 - y2);
	if (dy > dx) {
		swap(&x1, &y1);
		swap(&x2, &y2);
		swap(&dx, &dy);
		slopegt1 = true;
	}
	if (x1 > x2) {
		swap(&x1, &x2);
		swap(&y1, &y2);
		reverse = true;
	}
	if (y1 > y2)
		incry = -1;
	else
		incry = 1;

	int dyx = dy - dx;
	d = dy + dyx;
	incre = dy + dy;
	incrne = dyx + dyx;

	// push first point
	if (slopegt1) {
		points.push_back(Point(y1, x1));
	} else {
		points.push_back(Point(x1, y1));
	}

	while (x1 < x2) {
		if (d <= 0) {
			d += incre;
		} else {
			d += incrne;
			y1 += incry;
		}
		x1++;
		if (slopegt1) {
			points.push_back(Point(y1, x1));
		} else {
			points.push_back(Point(x1, y1));
		}
	}
	if (reverse) {
		std::reverse(points.begin(), points.end());
	}
}

Angle Line::getSlope()
{
	int deltax = end.x - start.x;
	// we have to mirror since pixel coordinate system is left handed
	int deltay = end.y - start.y;
	return Angle::rad(atan2(deltay, deltax));
}

float Line::getLength()
{
	int deltax = end.x - start.x;
	int deltay = end.y - start.y;
	return sqrt(deltax * deltax + deltay * deltay);
}

bool Line::isValid()
{
	return start.x != 0 || start.y != 0 || end.x != 0 || end.y != 0;
}

/**
 * precision in deg
 */
int Line::checkSameDirection(int x1, int y1, int x2, int y2, const Angle &precision)
{
	Line line2(x1, y1, x2, y2);
	Angle m1 = getSlope();
	Angle m2 = line2.getSlope();
	if (m1.eq(m2, precision)) {
		return 0;
	}
	if (m1 < m2) {
		return -1;
	}
	return 1;
}

int Line::checkSameDirection(Line &line2, const Angle &precision)
{
	return checkSameDirection(line2.start.x, line2.start.y, line2.end.x, line2.end.y, precision);
}
