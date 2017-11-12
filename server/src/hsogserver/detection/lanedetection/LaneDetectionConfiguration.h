#pragma once

namespace taco
{
class LaneDetectionConfiguration
{
  public:
	// width and height of the image in pixel
	int height = 480;
	int width = 640;

	// how many pixels we stay away from bottom of image with scanning
	int yOffset = 2;
	// the focal point in the full image
	float focalPointX = 330;
	float focalPointY = 240;

	// the number of pixels at scan line distance for 1m
	int pixelPerMeter = 449;

	int rightLaneStartPos = 20;

	int heightCamera = 215;
	float focalVertical = 552.5;
	float focalHorizontal = 552.5;
	float xCenter = 319.5;
};
}
