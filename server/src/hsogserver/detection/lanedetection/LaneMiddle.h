#pragma once

#include <eigen3/Eigen/Core>

namespace taco
{
class LaneMiddle
{
  public:
	LaneMiddle(){};

	LaneMiddle(const LaneMiddle &source)
	{
		valid = source.valid;
		confidence = source.confidence;
		invalidSince = source.invalidSince;
		wantedX = source.wantedX;
		middleX = source.middleX;
		aheadLineY = source.aheadLineY;
		middleXCamera = source.middleXCamera;
		rightLineX = source.rightLineX;
		middleLineX = source.middleLineX;
		leftLineX = source.leftLineX;
		scanRightStartX = source.scanRightStartX;
		scanRightStartY = source.scanRightStartY;
		scanRightEndX = source.scanRightEndX;
		inCrossing = source.inCrossing;
	};

	// true if we detected lane middle in the last image
	bool valid = false;
	// the confidence we have in the detection
	float confidence = 0.0;
	// count since how many images we did not detect lane middle
	int invalidSince = 0;
	// the desired x pixel coordinate we want the lane middle to be
	int wantedX;
	// the calculated x pixel coordinate of the lane middle
	int middleX;
	// the y pixel coordinate of the line ahead
	int aheadLineY;
	// the calculated lane middle in camera coordinate system
	Eigen::Vector3d middleXCamera;

	// the x pixel coordinate of the right line at the lane following scan line
	int rightLineX;
	// the x pixel coordinate of the middle or left line at the lane following scan line
	int middleLineX;
	// the x pixel coordinate of the middle or left line at the lane following scan line
	int leftLineX;
	// the x pixel coordinate for starting to scan for right line
	int scanRightStartX;
	// the y pixel coordinate for starting to scan for right line
	int scanRightStartY;
	// the x pixel coordinate for ending to scan for right line
	int scanRightEndX;
	// true if we believe we are in a crossing
	bool inCrossing = false;
};
}
