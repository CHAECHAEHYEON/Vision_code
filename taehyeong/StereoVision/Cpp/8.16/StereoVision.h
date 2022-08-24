#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <string>


using namespace std;
using namespace cv;

class StereoVision
{

public:

	StereoVision(float baseline, float focal_pixels)
	: baseline(baseline),focal_pixels(focal_pixels) {}

	// Calibrate the Frames
	Mat undistortFrame(Mat& frame);

	// Add HSV filter - Filter out / Segment the red ball
	Mat add_HSV_filter(Mat& frame, int camera);

	// Find the Cirle/Ball - Only find the largest one - Reduce false positives
	Point find_ball(Mat& frame, Mat& mask);

	// Calculate the depth to the ball - formulas in the video
	float find_depth(Point circleLeft, Point circleRight, Mat& leftFrame, Mat& rightFrame, float alpha, float beta);



private:

	float baseline = 0;
	float focal_pixels = 0;

};

