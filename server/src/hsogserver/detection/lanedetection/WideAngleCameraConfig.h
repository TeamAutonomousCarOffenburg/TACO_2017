#pragma once

namespace taco
{
class WideAngleCameraConfig : public LaneDetectionConfiguration
{
  public:
	WideAngleCameraConfig()
	{
		height = 330;
		width = 1240;
		yOffset = 25;
		xCenter = 619;
		// measured 4/10/2017
		focalPointX = 586;
		focalPointY = 187;
		pixelPerMeter = 444;

		heightCamera = 215;
		focalVertical = 400.0;
		focalHorizontal = 400.0;
	}
};
}
