
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>

#pragma once

class EdgeDetector
{
  public:
	static cv::Mat detectEdges(cv::Mat &frame, int width, int height)
	{
		cv::Size frame_size = cv::Size(width, height);
		cv::Mat edges(frame_size, CV_8UC1);
		cv::Mat grey(frame_size, CV_8UC1);
		// cut off upper half
		cv::Rect rect(0, frame_size.height, frame_size.width, frame_size.height);
		cv::Mat temp_frame = frame(rect);
		cv::cvtColor(temp_frame, grey, cv::COLOR_BGR2GRAY); // convert to grayscale

		// Perform a Gaussian blur ( Convolving with 5 X 5 Gaussian) & detect edges
		cv::GaussianBlur(grey, grey, cv::Size(5, 5), 0);

		int cannyMinThreshold = 100;
		int cannyMaxThreshold = 150;
		cv::Canny(grey, edges, cannyMinThreshold, cannyMaxThreshold);

		return edges;
	}
};
