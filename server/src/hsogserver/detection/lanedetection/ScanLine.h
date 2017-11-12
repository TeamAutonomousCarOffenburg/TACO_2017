#pragma once

#include "LaneDetectionConfiguration.h"
#include "Line.h"
#include "PixelToCamera.h"
#include <opencv2/core/core.hpp>

namespace taco
{
class ScanLine
{
  public:
	ScanLine(int x1, int y1, int x2, int y2);
	virtual ~ScanLine();

	/**
	 * Searches for the first two set pixel that have a distance we expect from a line on a canny edge.
	 * Returns the x pixel coordinate of the first found pixel
	 * mat: the image in which to search
	 * maxWidth the maximal distance in pixel the to sides of the line may have
	 */
	cv::Point findFirstDoubleLinePixel(cv::Mat &mat, int maxWidth);

	/**
	 * Searches for the first set pixel on the scan line
	 * mat: the image in which to search
	 */
	cv::Point findFirstPixel(cv::Mat &mat);
	cv::Point findNextPixel(cv::Mat &mat);

  private:
	std::vector<cv::Point> _points;
	Line _line;
	int currentPoint;
	int dirDelta[8][6] = {
			//
			{0, -1, -1, -1, 1, -1}, // up
			{-1, -1, -1, 0, 0, -1}, // left up
			{-1, 0, -1, 1, -1, -1}, // left
			{-1, 1, 0, 1, -1, 0},   // left down
			{0, 1, 1, 1, -1, 1},	// down
			{1, 1, 1, 0, 0, 1},		// right down
			{1, 0, 1, -1, 1, 1},	// right
			{1, -1, 0, -1, 1, 0}	// right up
	};

	bool gotoNextPoint(cv::Mat &mat, cv::Point &currentPoint, int direction);
};
}
