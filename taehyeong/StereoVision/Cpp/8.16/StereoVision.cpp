#include "StereoVision.h"


Mat StereoVision::undistortFrame(Mat& frame)
{

	Mat cameraMatrix = Mat::eye(3,3,CV_64FC1);
	Mat distCoeffs = Mat::zeros(1,5,CV_64FC1);
	Mat dst;
	cameraMatrix = (Mat1d(3,3) << 509.5140, 0, 321.9972, 0, 510.5093, 258.7457, 0., 0., 1. );
	distCoeffs = (Mat1d(1,5) << 0.0891, -0.1673, 0., 0., 0.);
	// newCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distortionParameters, { frame.cols, frame.rows }, 1);
	
	undistort(frame, dst, cameraMatrix, distCoeffs);

	// Can be calibrated as in the other video on the channel

	/*// Precompute lens correction interpolation
	Mat mapX, mapY;
	undistort(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1,
		mapX, mapY);

	// Show lens corrected images
	std::cout << std::string(f) << std::endl;

	cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);

	cv::Mat imgUndistorted;
	// 5. Remap the image using the precomputed interpolation maps.
	cv::remap(img, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);*/
	return dst;
}



Mat StereoVision::add_HSV_filter(Mat& frame, int camera) {

	// Blurring the frame to reduce noise
	// GaussianBlur(frame, frame, { 5,5 }, 0);

	// Convert to HSV
	cvtColor(frame, frame, COLOR_BGR2HSV);

	Mat mask;
	// logitech c930e

	vector<int> lowerLimitRedRight = { 10, 170, 120 };     // Lower limit for white ball
	vector<int> upperLimitRedRight = { 40, 255, 255 };	 // Upper limit for white ball
	vector<int> lowerLimitRedLeft = { 10, 170, 120 };     // Lower limit for white ball
	vector<int> upperLimitRedLeft = { 40, 255, 255 };	 // Upper limit for white ball

	// vector<int> lowerLimitRedRight = { 0, 0, 170 };     // Lower limit for white ball
	// vector<int> upperLimitRedRight = { 255, 100, 255 };	 // Upper limit for white ball
	// vector<int> lowerLimitRedLeft = { 0, 0, 170 };     // Lower limit for white ball
	// vector<int> upperLimitRedLeft = { 255, 100, 255 };	 // Upper limit for white ball

	if (camera == 1) {
		inRange(frame, lowerLimitRedRight, upperLimitRedRight, mask);
	} else {
		inRange(frame, lowerLimitRedLeft, upperLimitRedLeft, mask);
	}

	/*vector<int> lowerLimitBlue = { 140,106,0 };     // Lower limit for blue ball
	vector<int> upperLimitBlue = { 255,255,255 };	 // Upper limit for blue ball
	*/

	erode(mask, mask, (3, 3));
	dilate(mask, mask, (3, 3));

	return mask;
}



Point StereoVision::find_ball(Mat& frame, Mat& mask) {

	vector<vector<Point> > contours;

	findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	// Sort the contours to find the biggest one
	sort(contours.begin(), contours.end(), [](const vector<Point>& c1, const vector<Point>& c2) {
		return contourArea(c1, false) < contourArea(c2, false);
	});

	if (contours.size() > 0) {

		vector<Point> largestContour = contours[contours.size() - 1];
		Point2f center;
		float radius;
		minEnclosingCircle(largestContour, center, radius);
		Moments m = moments(largestContour);
		Point centerPoint(m.m10 / m.m00, m.m01 / m.m00);

		// Only preceed if the radius is grater than a minimum threshold
		if (radius > 10) {
			// Draw the circle and centroid on the frame
			circle(frame, center, int(radius), (0, 255, 255), 2);
			circle(frame, centerPoint, 5, (0, 0, 255), -1);
		}

		return centerPoint;
	}

	return { 0,0 };
}



float StereoVision::find_depth(Point circleLeft, Point circleRight, Mat& leftFrame, Mat& rightFrame, float alpha, float beta) 
{	
	float x_0 = 0;
	float y_0 = 0;
	if ((rightFrame.cols == leftFrame.cols) && (rightFrame.rows == leftFrame.rows))
	{	
		x_0 = rightFrame.cols/2;
		y_0 = rightFrame.rows/2;
	}
	else {
		cout << "Left and Right Camera frames do not have the same pixel width" << endl;	
	}

	float xLeft = circleLeft.x;
	float xRight = circleRight.x;
	float yLeft = circleLeft.y;
	float yRight = circleRight.y;


	float realX = 0;
	float realY = 0;
	float realZ = 0;
	float distance = 0;

	
	// if((xLeft < x_0) && (xRight < x_0))
	// {
	// 	realX = (float)baseline/(1-(x_0-xRight)/(x_0-xLeft));
	// 	realZ = abs(-realX*focal_pixels/(x_0-xLeft));
	// }
	// else if((xLeft > x_0) && (xRight < x_0))
	// {
	// 	realX = (float)baseline/(1 + (x_0 - xRight)/(xLeft - x_0));
	// 	realZ = abs(realX*focal_pixels/(xLeft-x_0));
	// }
	// else if((xLeft > x_0) && (xRight > x_0))
	// {
	// 	realX = (float)baseline/(1 - (xRight - x_0)/(xLeft - x_0));
	// 	realZ = abs(realX*focal_pixels/(xLeft-x_0));
	// }
	// else{
	// 	realX = 0;
	// 	realZ = 0;
	// }

	if(xLeft != x_0)
	{
		realX = (float)baseline/(1 - (x_0 - xRight)/(x_0 - xLeft));
		realZ = abs(realX*focal_pixels/(x_0 - xLeft));
	}
	else if(xRight != x_0)
	{
		realX = -(float)baseline/(1 - (x_0 - xLeft)/(x_0 - xRight));
		realZ = abs(realX*focal_pixels/(x_0 - xRight));
		realX = realX + (float)baseline; //왼쪽 카메라 기준
	}
	else
	{
		realX = 0;
		realY = 0;
	}
	realY = realZ*(2*y_0-yLeft-yRight)/(2*focal_pixels);

	distance = sqrt(pow(realX,2)+pow(realY,2) + pow(realZ,2));

	//1번 식--------------------------------------------------------------

	//alpha = 카메라 머리 숙인 각도, beta = erp 헤딩으로부터 카메라 각도
	alpha = alpha * CV_PI / 180;
	beta = beta * CV_PI / 180;
	
	float seta = 0;
	seta = atan(realY/realZ) - alpha;
	realZ = sqrt(pow(realZ,2)+pow(realY,2))*cos(seta);
	realY = sqrt(pow(realZ,2)+pow(realY,2))*sin(seta);

	float realZ_copy = realZ;
	float realX_copy = realX;

	float gama = atan(realX/realZ) + beta;
	realZ = sqrt(pow(realZ_copy,2)+pow(realX_copy,2))*cos(gama);
	realX = sqrt(pow(realZ_copy,2)+pow(realX_copy,2))*sin(gama);
	
	float angle = 0;
	angle = atan(realX/realZ)*180/CV_PI;

	cout << "realX : " << realX << "	realZ : " << realZ << "		angle : " << angle << endl;
	//2번 식--------------------------------------------------------------
	// float realX_alpha = 0;
	// float realZ_alpha = 0;
	// // alpha_2는 erp헤딩으로부터 물체까지 각도
	// float alpha_2 = 0;
	// alpha_2 = beta*CV_PI/180 + atan(realX/(realZ*cos(alpha*CV_PI/180))); 

	// cout << "alpha_2 : " << alpha_2*180/CV_PI << endl;
	// realX_alpha = sqrt(pow(realX,2)+pow(realZ*cos(alpha*CV_PI/180),2))*cos(CV_PI/2 - alpha_2);
	// realZ_alpha = sqrt(pow(realX,2)+pow(realZ*cos(alpha*CV_PI/180),2))*cos(alpha_2);

	// cout << " realX_alpha : " << realX_alpha << "  realZ_alpha : " << realZ_alpha << endl;
	//---------------------------------------------------------------------

	return distance;
}