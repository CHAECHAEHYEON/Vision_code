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

	return dst;
}



Mat StereoVision::add_HSV_filter(Mat& frame, int camera) {

	// Blurring the frame to reduce noise
	// GaussianBlur(frame, frame, { 5,5 }, 0);

	// Convert to HSV
	cvtColor(frame, frame, COLOR_BGR2HSV);

	Mat mask;

	// logitech c930e
	// vector<int> lowerLimitRedRight = { 10, 160, 100 };     // Lower limit for yellow
	// vector<int> upperLimitRedRight = { 40, 255, 255 };	 // Upper limit for yellow
	// vector<int> lowerLimitRedLeft = { 10, 160, 100 };     // Lower limit for yellow
	// vector<int> upperLimitRedLeft = { 40, 255, 255 };	 // Upper limit for yellow

	vector<int> lowerLimitRedRight = { 0, 0, 170 };     // Lower limit for white
	vector<int> upperLimitRedRight = { 255, 100, 255 };	 // Upper limit for white
	vector<int> lowerLimitRedLeft = { 0, 0, 170 };     // Lower limit for white
	vector<int> upperLimitRedLeft = { 255, 100, 255 };	 // Upper limit for white

	if (camera == 1) {
		inRange(frame, lowerLimitRedRight, upperLimitRedRight, mask);
	} else {
		inRange(frame, lowerLimitRedLeft, upperLimitRedLeft, mask);
	}

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


void StereoVision::line_symmetry(Mat& frame, int camera)
{
	//left camera, right camera 한 평면 위에 놓는 작업

	Mat img = frame.clone();

	rectangle(img, Rect(Point(635,0),Point(645,720)),Scalar(0,255,0),2,4,0);
	rectangle(img, Rect(Point(0,680),Point(1280,690)),Scalar(0,255,0),2,4,0);
	rectangle(img, Rect(Point(0,640),Point(1280,650)),Scalar(0,255,0),2,4,0);
	rectangle(img, Rect(Point(0,600),Point(1280,610)),Scalar(0,255,0),2,4,0);
	rectangle(img, Rect(Point(0,560),Point(1280,570)),Scalar(0,255,0),2,4,0);
	rectangle(img, Rect(Point(0,520),Point(1280,530)),Scalar(0,255,0),2,4,0);
	rectangle(img, Rect(Point(0,480),Point(1280,490)),Scalar(0,255,0),2,4,0);
	rectangle(img, Rect(Point(0,360),Point(1280,370)),Scalar(0,255,0),2,4,0);

	if(camera == 0)
	{
		imshow("left_line_symmetry", img);
	}
	else
	{
		imshow("right_line_symmetry", img);
	}
}

float *StereoVision::mass_center(Mat &img, float array[], int camera)
{
	Mat mask = img.clone();
    // contours
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

    Scalar color(rand() & 255, rand() & 255, rand() & 255);

    int lrgctridx = 0;
    int minarea = 1000000;
    double nowarea = 0;

    for (int i = 0; i < contours.size(); i++)
    {
        nowarea = contourArea(contours[i]);
        // 모든 외각선의 사이즈 출력
        // cout << "contour idx = " << i << " " << "size = " << nowarea << endl;

        if ((nowarea > 25000) && (nowarea < 150000))
        {
            if(nowarea < minarea)
            {
                minarea = nowarea;
                lrgctridx = i;
            }
        }
    }
	// cout << "contour size = " << minarea << endl;

    if(contours.size() > 0)
    {
        Mat drawing = Mat::zeros( mask.size(), CV_8UC3 );

        // 모든 외각선 그리기
        // for(int idx = 0; idx >= 0; idx = hierarchy[idx][0])
        // {
        //     drawContours( drawing, contours, idx, color, 2, LINE_8, hierarchy);
        // }

        // 특정한 외각선만 그리기
        drawContours( drawing, contours, lrgctridx, color, 2, LINE_8, hierarchy);

        vector<Point> hull;
        convexHull(Mat(contours[lrgctridx]), hull, false);

        vector<vector<Point>> fake_hull;
        fake_hull.push_back(hull);
        drawContours(drawing, fake_hull, 0, color, 2, LINE_8);

        int top_x_left = hull[0].x;
	    int top_y_left = hull[0].y;
        int top_num_left = 0;

        int bottom_x_left = hull[0].x;
        int bottom_y_left = hull[0].y;
        int bottom_num_left = 0;

        int top_x_right = hull[0].x;
	    int top_y_right = hull[0].y;
        int top_num_right = 0;

        int bottom_x_right = hull[0].x;
        int bottom_y_right = hull[0].y;
        int bottom_num_right = 0;

        for(int i = 0; i < hull.size(); i++)
        {
            if(hull[i].y < top_y_left)
            {
                top_x_left = hull[i].x;
                top_y_left = hull[i].y;
                top_num_left = i;
            }
            if(sqrt(pow(hull[i].x - img.cols*7/8, 2) + pow(hull[i].y - 0, 2)) <= sqrt(pow(top_x_right - img.cols*7/8, 2) + pow(top_y_right - 0, 2)))
            {
                top_x_right = hull[i].x;
                top_y_right = hull[i].y;
                top_num_right = i;
            }
            if((1*sqrt(pow(hull[i].x - 0, 2) + pow(hull[i].y - img.rows, 2)) + 9*hull[i].x) < (1*sqrt(pow(bottom_x_left - 0, 2) + pow(bottom_y_left - img.rows, 2)) + 9*bottom_x_left))
            {
                bottom_x_left = hull[i].x;
                bottom_y_left = hull[i].y;
                bottom_num_left = i;
            }
            if(hull[i].y > bottom_y_right)
            {
                bottom_x_right = hull[i].x;
                bottom_y_right = hull[i].y;
                bottom_num_right = i;
            }
        }

        float mean_top_x = (float)((top_x_left + top_x_right) / 2.0);
        float mean_top_y = (float)((top_y_left + top_y_right) / 2.0);
		float mean_mid_x = (float)((top_x_left+top_x_right+bottom_x_left+bottom_x_right)/4.0);
		float mean_mid_y = (float)((top_y_left+top_y_right+bottom_y_left+bottom_y_right)/4.0);
        float mean_bottom_x = (float)((bottom_x_left + bottom_x_right) / 2.0);
        float mean_bottom_y = (float)((bottom_y_left + bottom_y_right) / 2.0);


        circle(drawing, Point((int)mean_top_x, (int)mean_top_y), 10, Scalar(0, 0, 255), -1);
		circle(drawing, Point((int)mean_mid_x, (int)mean_mid_y), 10, Scalar(0, 255, 0), -1);
        circle(drawing, Point((int)mean_bottom_x, (int)mean_bottom_y), 10, Scalar(255, 0, 0), -1);
        
        circle(drawing, Point(top_x_left, top_y_left), 4, Scalar(255, 255, 255), -1);
        circle(drawing, Point(top_x_right, top_y_right), 4, Scalar(255, 255, 255), -1);
        circle(drawing, Point(bottom_x_left, bottom_y_left), 4, Scalar(255, 255, 255), -1);
        circle(drawing, Point(bottom_x_right, bottom_y_right), 4, Scalar(255, 255, 255), -1);


        array[0] = mean_bottom_x;
		array[1] = mean_bottom_y;
		array[2] = mean_mid_x;
		array[3] = mean_mid_y;
		array[4] = mean_top_x;
		array[5] = mean_top_y;

		if(camera == 0)
		{
			imshow("Left_drawing", drawing);
		}
		else
		{
			imshow("right_drawing", drawing);
		}
		
		return array;
    }
    else
    {
        return array;
    }
}

Point2f StereoVision::find_XZ(Point2f left_point, Point2f right_point, Mat& leftFrame, Mat& rightFrame)
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

	float xLeft = left_point.x;
	float xRight = right_point.x;
	float yLeft = left_point.y;
	float yRight = right_point.y;

	float realX = 0;
	float realY = 0;
	float realZ = 0;
	float distance = 0;

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

	// cout << " realX : " << realX << "   realY : "<< realY << "     realZ : " << realZ << endl;
	// cout << " distance : " << distance << endl;
	
	// cout << " 영점 조절 : " << realX << endl;

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

	return {realZ, realX};
}