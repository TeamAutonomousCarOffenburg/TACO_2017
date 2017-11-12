#include "LaneDetectionRenderer.h"

using namespace taco;
using namespace std;
using namespace cv;

void LaneDetectionRenderer::drawLine(Mat &image, Line line2, Scalar &color)
{
	line(image, line2.start, line2.end, color);
}

Mat LaneDetectionRenderer::draw(int width, int height, const Mat &image, LaneAssist laneAssist)
{
	Rect rect(0, height / 2, width, height / 2);
	Mat temp_frame = image(rect);
	Scalar red = Scalar(0, 0, 255);
	Scalar green = Scalar(0, 255, 0);
	Scalar blue = Scalar(255, 0, 0);
	Scalar orange = Scalar(0, 130, 200);

	if (laneAssist.laneMiddle.valid) {
		int x = laneAssist.laneMiddle.rightLineX;
		int y = laneAssist.laneMiddle.scanRightStartY;
		drawLine(temp_frame, Line(laneAssist.laneMiddle.middleX, y - 5, laneAssist.laneMiddle.middleX, y + 5), red);
		drawLine(temp_frame, Line(laneAssist.laneMiddle.wantedX, y - 5, laneAssist.laneMiddle.wantedX, y + 5), green);
		if (x >= 0) {
			drawLine(temp_frame, Line(x, y - 5, x, y + 5), blue);
		}
		x = laneAssist.laneMiddle.middleLineX;
		if (x >= 0) {
			drawLine(temp_frame, Line(x, y - 5, x, y + 5), blue);
		}
		x = laneAssist.laneMiddle.leftLineX;
		if (x >= 0) {
			drawLine(temp_frame, Line(x, y - 5, x, y + 5), blue);
		}
		x = laneAssist.laneMiddle.middleX;
		y = laneAssist.laneMiddle.aheadLineY;
		if (y >= 0) {
			drawLine(temp_frame, Line(x - 5, y, x + 5, y), blue);
		}
	}

	Line scan = Line(laneAssist.laneMiddle.scanRightStartX, laneAssist.laneMiddle.scanRightStartY,
			laneAssist.laneMiddle.scanRightEndX, laneAssist.laneMiddle.scanRightStartY);
	drawLine(temp_frame, scan, orange);

	return temp_frame;
}