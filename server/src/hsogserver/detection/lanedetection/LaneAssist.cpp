#include "LaneAssist.h"
#include "../hsogserver/utils/geometry/Line2D.h"

using namespace taco;
using namespace cv;
using namespace Eigen;
using namespace std;

LaneAssist::LaneAssist(LaneDetectionConfiguration cfg) : _cfg(cfg)
{
}

LaneAssist::~LaneAssist()
{
}
void LaneAssist::detectLaneMiddle(cv::Mat &frame)
{
	int x = static_cast<int>(_cfg.width * 0.1);
	LaneMiddle result = laneMiddle;
	result.valid = false;
	double bestConfidence = 0.0;
	for (; x < _cfg.width * 0.7; x += 50) {
		LaneMiddle check(laneMiddle);
		detection(frame, check, x);
		if (check.confidence > bestConfidence) {
			result = check;
			bestConfidence = check.confidence;
		}
	}

	laneMiddle = result;
}

void LaneAssist::detection(cv::Mat &frame, LaneMiddle &resultMiddle, int scanRightStartX)
{
	int y = 118;
	resultMiddle.wantedX = static_cast<int>(_cfg.focalPointX);
	int result = resultMiddle.middleX;

	int pixelRightX = reactiveDetectRight(frame, resultMiddle, scanRightStartX, y);
	int pixelMiddleX = -1;
	int pixelLeftX = -1;
	float confidence = 0;

	if (resultMiddle.inCrossing) {
		// inside crossings we do not detect
		result = resultMiddle.wantedX;

	} else {
		// -5 to have the middle of the midline
		pixelMiddleX = reactiveDetectMiddle(frame, resultMiddle, y, pixelRightX) - 5;
		pixelLeftX = reactiveDetectLeft(frame, y, pixelRightX, pixelMiddleX);

		// in crossings we might have mixed up mid and left with right and mid line
		if (abs(pixelRightX - resultMiddle.middleLineX) < 25 && abs(pixelMiddleX - resultMiddle.leftLineX) < 25) {
			pixelLeftX = pixelMiddleX;
			pixelMiddleX = pixelRightX;
			pixelRightX = -1;
		}

		checkPlausibility(resultMiddle, pixelRightX, pixelMiddleX, pixelLeftX);
		prepareResult(resultMiddle, pixelRightX, pixelMiddleX, pixelLeftX, result, confidence);
	}

	if (confidence < 0.01) {
		// we do not see anything
		resultMiddle.valid = false;
		resultMiddle.invalidSince++;
		resultMiddle.rightLineX = -1;
		resultMiddle.middleLineX = -1;
		resultMiddle.leftLineX = -1;
		resultMiddle.confidence = 0;
		resultMiddle.aheadLineY = -1;
	} else {
		resultMiddle.valid = true;
		resultMiddle.invalidSince = 0;
		resultMiddle.middleX = result;
		resultMiddle.aheadLineY = findLineAhead(frame, result, y);
		Vector3d cam = _pixelToCamera->pixelToCamera(Point(result, y));
		resultMiddle.middleXCamera = cam;
		resultMiddle.rightLineX = pixelRightX;
		resultMiddle.middleLineX = pixelMiddleX;
		resultMiddle.leftLineX = pixelLeftX;
		resultMiddle.confidence = confidence;
	}

	//    cout << "y: " << y << " lane: " << result << " confidence: " << confidence << " left: " << pixelLeftX
	//		 << " middle: " << pixelMiddleX << " right: " << pixelRightX << " aheadY: " << resultMiddle.aheadLineY
	//		 << " inCrossing: " << resultMiddle.inCrossing << endl;
}

int LaneAssist::reactiveDetectRight(cv::Mat &frame, LaneMiddle &resultMiddle, int scanLineStartX, int scanLineY)
{
	int maxX = _cfg.width - 2;
	resultMiddle.scanRightStartX = scanLineStartX;
	resultMiddle.scanRightEndX = maxX;

	resultMiddle.scanRightStartY = scanLineY;
	ScanLine rightScan(resultMiddle.scanRightStartX, scanLineY, resultMiddle.scanRightEndX, scanLineY);
	int pixelRightX = rightScan.findFirstDoubleLinePixel(frame, 35).x;

	if ((pixelRightX < 0 || pixelRightX > maxX * 0.9) && resultMiddle.rightLineX > 0 &&
			resultMiddle.rightLineX < maxX * 0.9) {
		// we saw the right line in the middle, but now have lost it? Seems to be a crossing
		resultMiddle.inCrossing = true;
		pixelRightX = -1;
	}

	if (resultMiddle.inCrossing && pixelRightX > 0 && pixelRightX < maxX * 0.8) {
		resultMiddle.inCrossing = false;
	}
	return pixelRightX;
}

int LaneAssist::reactiveDetectMiddle(cv::Mat &frame, LaneMiddle &resultMiddle, int y, int rightX)
{
	// search lines to the left starting from the middle
	int laneWidth = static_cast<int>(_cfg.pixelPerMeter * 0.225);
	int minx = 1;
	int maxX = _cfg.width - 2;
	int scanLeftX1 = resultMiddle.scanRightStartX;
	if (rightX > 0) {
		scanLeftX1 = rightX - laneWidth;
		checkRange(scanLeftX1, minx, maxX);
	}
	int scanLeftX2 = scanLeftX1 - 2 * laneWidth;
	checkRange(scanLeftX2, minx, maxX);

	int pixelX1 = -1;
	int pixelY1 = y;
	findMiddle(frame, pixelX1, pixelY1, scanLeftX1, scanLeftX2, y, -1);

	if (pixelX1 > 0 && pixelY1 == y) {
		// found point on scanline
		return pixelX1;
	}

	int pixelX2 = -1;
	int pixelY2 = y;
	findMiddle(frame, pixelX2, pixelY2, scanLeftX1, scanLeftX2, y + 5, 1);

	if (pixelX2 < 0) {
		if (pixelX1 < 0) {
			// nothing found
			return -1;
		}
		// did not search for lower or did not find lower
		if (resultMiddle.valid && abs(resultMiddle.middleLineX - pixelX1) < 50) {
			// we have moved upward to find middle so interpolate between old and new
			pixelX1 = (int) ((pixelX1 + resultMiddle.middleLineX) * 0.5);
			return pixelX1;
		}
		return -1;

	} else if (abs(pixelX1 - pixelX2) > 50) {
		// two points do not match together
		return -1;
	}

	Line2D line(pixelX1, pixelY1, pixelX2, pixelY2);
	int result = (int) line.xValue(y);

	return result;
}

void LaneAssist::findMiddle(cv::Mat &frame, int &pixelX, int &pixelY, int scanX1, int scanX2, int y, int direction)
{
	int minx = 1;
	int maxX = _cfg.width - 2;
	int miny = 1;
	int maxy = _cfg.height / 2 - _cfg.yOffset;
	pixelX = -1;
	int deltaY = 0;
	int y1 = y;
	int x1 = scanX1;
	int x2 = scanX2;
	int deltaX = 0;
	int maxDelta = 50;
	if (x1 < maxX * 0.4) {
		deltaX = -5;
		maxDelta = 20;
	} else if (x1 > maxX * 0.65) {
		deltaX = 5;
		maxDelta = 20;
	}
	while (pixelX < 0 && deltaY < maxDelta) {
		ScanLine scan2(x1, y1, x2, y1);
		pixelX = scan2.findFirstPixel(frame).x;
		pixelY = y1;

		deltaY += 5;
		y1 = y + deltaY * direction;
		checkRange(y1, miny, maxy);
		x1 += deltaX;
		checkRange(x1, minx, maxX);
		x2 += deltaX;
		checkRange(x2, minx, maxX);
	}
}

int LaneAssist::reactiveDetectLeft(cv::Mat &frame, int y, int rightX, int middleX)
{
	int laneWidth = static_cast<int>(_cfg.pixelPerMeter * 0.225);
	int x1 = middleX - laneWidth;
	if (x1 < 1) {
		x1 = rightX - 3 * laneWidth;
		if (x1 < 1) {
			x1 = 1;
		}
	}

	ScanLine leftScan(x1, y, 1, y);
	int pixelLeftX = leftScan.findFirstDoubleLinePixel(frame, 30).x;

	return pixelLeftX;
}

void LaneAssist::setPixelToCamera(PixelToCamera::ConstPtr pixelToCamera)
{
	_pixelToCamera = pixelToCamera;
}

void LaneAssist::checkRange(int &value, int min, int max)
{
	if (value < min) {
		value = min;
	} else if (value > max) {
		value = max;
	}
}

void LaneAssist::checkPlausibility(LaneMiddle &resultMiddle, int &pixelRightX, int &pixelMiddleX, int &pixelLeftX)
{
	int deltaR = 100;
	int deltaM = 100;
	int deltaL = 100;
	// do not allow jumps
	if (resultMiddle.valid) {
		checkJump(pixelRightX, resultMiddle.rightLineX, deltaR);
		checkJump(pixelMiddleX, resultMiddle.middleLineX, deltaM);
		checkJump(pixelLeftX, resultMiddle.leftLineX, deltaL);
	}

	// check relative positions
	int delta1 = pixelRightX - pixelMiddleX;
	int delta2 = pixelMiddleX - pixelLeftX;
	int delta3 = pixelRightX - pixelLeftX;
	int minDelta = 120;
	int maxDelta = 400;

	if (pixelRightX > 0 && pixelMiddleX > 0) {
		if (delta1 < minDelta || delta1 > maxDelta) {
			if (abs(deltaR) < abs(deltaM)) {
				pixelMiddleX = -1;
			} else {
				pixelRightX = -1;
			}
			// cout << "unplausible x-m: " << delta1 << endl;
		}
	}

	if (pixelMiddleX > 0 && pixelLeftX > 0) {
		if (delta2 < minDelta || delta2 > maxDelta) {
			if (abs(deltaL) < abs(deltaM)) {
				pixelMiddleX = -1;
			} else {
				pixelLeftX = -1;
			}
			// cout << "unplausible m-l: " << delta2 << " deltam: " << deltaM << " deltal: " << deltaL << endl;
		}
	}

	if (pixelRightX > 0 && pixelLeftX > 0) {
		if (delta3 < minDelta * 2 || delta3 > maxDelta * 2) {
			if (abs(deltaR) < abs(deltaL)) {
				pixelLeftX = -1;
			} else {
				pixelRightX = -1;
			}
			// cout << "unplausible x-l: " << delta3 << endl;
		}
	}
}

void LaneAssist::checkJump(int &pixelX, int oldX, int &delta)
{
	if (pixelX > 0 && oldX > 0) {
		delta = pixelX - oldX;
	}
}

void LaneAssist::prepareResult(const LaneMiddle &resultMiddle, int pixelRightX, int pixelMiddleX, int pixelLeftX,
		int &result, float &confidence) const
{
	int laneWidth = static_cast<int>(_cfg.pixelPerMeter * 0.225);

	if (pixelRightX > 0 && pixelMiddleX > 0 && pixelLeftX > 0) {
		// see all three
		result = (int) (((pixelRightX + pixelMiddleX) * 0.5 + pixelMiddleX + (pixelMiddleX - pixelLeftX) * 0.5 +
								pixelMiddleX + (pixelRightX - pixelLeftX) * 0.25) /
						3.0);
		confidence = getConfidenceFromDelta(1.0, pixelRightX - pixelMiddleX, pixelMiddleX - pixelLeftX);

	} else if (pixelRightX > 0 && pixelMiddleX > 0) {
		result = (int) ((pixelRightX + pixelMiddleX) * 0.5);
		confidence = getConfidenceFromDelta(0.8, pixelRightX - pixelMiddleX, laneWidth * 2);

	} else if (pixelRightX > 0 && pixelLeftX > 0) {
		result = (int) (pixelRightX - (pixelRightX - pixelLeftX) * 0.25);
		confidence = getConfidenceFromDelta(0.8, pixelRightX - pixelLeftX, laneWidth * 4);

	} else if (pixelMiddleX > 0 && pixelLeftX > 0) {
		result = (int) (pixelMiddleX + (pixelMiddleX - pixelLeftX) * 0.5);
		confidence = getConfidenceFromDelta(0.75, pixelMiddleX - pixelLeftX, laneWidth * 2);

	} else if (pixelRightX > 0) {
		// only see right
		if (resultMiddle.valid) {
			result = resultMiddle.middleX + (pixelRightX - resultMiddle.rightLineX);
		} else {
			result = pixelRightX - laneWidth;
		}
		confidence = 0.4;

	} else if (pixelMiddleX > 0) {
		// only see middle
		if (resultMiddle.valid) {
			result = resultMiddle.middleX + (pixelMiddleX - resultMiddle.middleLineX);
		} else {
			result = pixelMiddleX + laneWidth;
		}
		confidence = 0.3;

	} else if (pixelLeftX > 0) {
		// only see left
		if (resultMiddle.valid) {
			result = resultMiddle.middleX + (pixelLeftX - resultMiddle.leftLineX);
		} else {
			result = pixelLeftX + laneWidth * 3;
		}
		confidence = 0.3;

	} else {
		confidence = 0;
	}

	if (result < 1) {
		result = 1;
	} else if (result > _cfg.width - 2) {
		result = _cfg.width - 2;
	}

	// reduce confidence by distance to expected middle
	int deltaExpected = abs(resultMiddle.wantedX - result);
	// cout << "delta expected: " << deltaExpected<< endl;
	if (deltaExpected > 800) {
		deltaExpected = 800;
	}
	confidence -= deltaExpected * 0.001;

	if (resultMiddle.valid && resultMiddle.confidence > 0.3) {
		// reduce confidence by distance to last detection
		int deltaLastTime = abs(resultMiddle.middleX - result);
		// cout << "delta expected: " << deltaLastTime << endl;
		if (deltaLastTime < 40) {
			deltaLastTime = 0;
		} else if (deltaLastTime > 100) {
			deltaLastTime = 100;
		}
		confidence -= deltaLastTime * 0.01;
	}
}

float LaneAssist::getConfidenceFromDelta(float confidence, int delta1, int delta2) const
{
	int delta = abs(delta2 - delta1);
	// cout << "delta left right: " << delta << endl;
	if (delta > 60) {
		delta = 60;
	}
	return confidence - delta * 0.005f;
}

int LaneAssist::findLineAhead(cv::Mat &frame, int x, int y)
{
	ScanLine leftScan(x, y, x, 1);
	return leftScan.findFirstDoubleLinePixel(frame, 30).y;
}
