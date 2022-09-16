#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <string>

#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h> // ros에서 opencv사용할 수 있게 함

#include "std_msgs/Int32.h" // Int32형 메시지 주고받을 수 있게 함
#include "std_msgs/Int16.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"

#include "std_msgs/Float64.h"
#include <image_transport/image_transport.h>
#include <sstream>


using namespace std;
using namespace cv;

class StereoVision
{

public:

	StereoVision(float baseline, float focal_pixels)
	: baseline(baseline),focal_pixels(focal_pixels){}

	// Calibrate the Frames
	Mat undistortFrame(Mat& frame);

	// Add HSV filter - Filter out / Segment the red ball
	Mat add_HSV_filter(Mat& frame, int camera);

	// Find the Cirle/Ball - Only find the largest one - Reduce false positives
	Point find_ball(Mat& frame, Mat& mask);

	void line_symmetry(Mat& frame, int camera);

	double* find_center(Mat &img, double array[], int camera);
	Point2d find_XZ(Point2d circleLeft, Point2d circleRight, Mat& leftFrame, Mat& rightFrame, float alpha, float beta);

	Mat adaptiveThreshold_parking(Mat src);


private:

	float baseline = 23;
	float focal_pixels = 800;
};

