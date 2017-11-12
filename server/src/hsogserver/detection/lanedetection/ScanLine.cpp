#include "ScanLine.h"
#include <iostream>

using namespace taco;
using namespace std;
using namespace cv;

ScanLine::ScanLine(int x1, int y1, int x2, int y2) : _line(x1, y1, x2, y2)
{
	_line.getLinePoints(_points);
	currentPoint = 0;
}

ScanLine::~ScanLine()
{
}

cv::Point ScanLine::findFirstDoubleLinePixel(cv::Mat &mat, int maxWidth)
{
	cv::Point p1 = findFirstPixel(mat);
	if (p1.x < 0) {
		// did not find any pixel
		return Point(-1, -1);
	}
	int i1 = currentPoint;
	cv::Point p2 = findNextPixel(mat);
	int i2 = currentPoint;
	while (p2.x >= 0) {
		int delta = abs(i2 - i1);
		if (delta > 1 && delta < maxWidth) {
			// found a valid line
			return p1;
		}
		p1 = p2;
		i1 = i2;
		p2 = findNextPixel(mat);
		i2 = currentPoint;
	}
	return Point(-1, -1);
}

cv::Point ScanLine::findFirstPixel(cv::Mat &mat)
{
	currentPoint = 0;
	return findNextPixel(mat);
}

cv::Point ScanLine::findNextPixel(cv::Mat &mat)
{
	for (int i = currentPoint; i < (int) _points.size(); i++) {
		currentPoint++;

		if (mat.at<uchar>(_points[i].y, _points[i].x) >= 255) {
			return _points[i];
		}
	}
	return Point(-1, -1);
}

bool ScanLine::gotoNextPoint(Mat &mat, Point &currentPoint, int direction)
{
	int x = currentPoint.x;
	int y = currentPoint.y;
	bool nextPixel = mat.at<uchar>(y + dirDelta[direction][1], x + dirDelta[direction][0]) >= 255;
	if (nextPixel) {
		x += dirDelta[direction][0];
		y += dirDelta[direction][1];
	} else {
		nextPixel = mat.at<uchar>(y + dirDelta[direction][3], x + dirDelta[direction][2]) >= 255;
		if (nextPixel) {
			x += dirDelta[direction][2];
			y += dirDelta[direction][3];
		} else {
			nextPixel = mat.at<uchar>(y + dirDelta[direction][5], x + dirDelta[direction][4]) >= 255;
			if (nextPixel) {
				x += dirDelta[direction][4];
				y += dirDelta[direction][5];
			}
		}
	}
	if (nextPixel) {
		currentPoint.x = x;
		currentPoint.y = y;
	}
	return nextPixel;
}
