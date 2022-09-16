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


/**
 * @brief HSV값으로 영상을 이진화하는 함수
 * 
 * @param frame 입력하고자 하는 화면
 * @param camera 카메라 번호 0 = 왼쪽, 1 = 오른쪽
 * @return Mat 
 */
Mat StereoVision::add_HSV_filter(Mat& frame, int camera) {

	// Blurring the frame to reduce noise
	// GaussianBlur(frame, frame, { 5,5 }, 0);

	// Convert to HSV
	// cvtColor(frame, frame, COLOR_BGR2HLS);
	cvtColor(frame, frame, COLOR_BGR2HSV);

	Mat mask;

	// logitech c930e
	vector<int> lowerLimitRedRight = { 10, 160, 100 };     // Lower limit for yellow
	vector<int> upperLimitRedRight = { 40, 255, 255 };	 // Upper limit for yellow
	vector<int> lowerLimitRedLeft = { 10, 160, 100 };     // Lower limit for yellow
	vector<int> upperLimitRedLeft = { 40, 255, 255 };	 // Upper limit for yellow


	// vector<int> lowerLimitRedRight = { 0, 0, 190 };     // Lower limit for white 세번째꺼만 바꾸는거
	// vector<int> upperLimitRedRight = { 255, 100, 255 };	 // Upper limit for white
	// vector<int> lowerLimitRedLeft = { 0, 0, 170 };     // Lower limit for white
	// vector<int> upperLimitRedLeft = { 255, 100, 255 };	 // Upper limit for white

	if (camera == 1) {
		inRange(frame, lowerLimitRedRight, upperLimitRedRight, mask);
	} else {
		inRange(frame, lowerLimitRedLeft, upperLimitRedLeft, mask);
	}

	erode(mask, mask, (3, 3));
	dilate(mask, mask, (3, 3));

	return mask;
}


/**
 * @brief 캘리용 함수
 * 
 * @param frame 입력 영상
 * @param mask 이진화된 영상 (HSV로 이진화하든 어쨌든 이진화된 영상)
 * @return Point 
 */
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

/**
 * @brief 스테레오 카메라 만들 때 사용하는 함수, 초기 제작할 때 수평을 맞추기 위해서 사용함
 * 
 * @param frame 입력 영상
 * @param camera 카메라 번호 0 = 왼쪽, 1 = 오른쪽
 */
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

/**
 * @brief 이진화한 영상에서 중심점을 찾고 하단, 중단, 상단의 x,y좌표를 double 형의 자료형으로 저장함
 * 
 * @param img 입력 영상 (이진화된 영상을 넣으면 됨)
 * @param array double자료형의 배열(바닥 x,y, 가운데 x,y, 상단 x,y 를 저장함)
 * @param camera 카메라 번호 0 = 왼쪽, 1 = 오른쪽
 * @return double* 
 */
double *StereoVision::find_center(Mat &img, double array[], int camera)
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
            // if(hull[i].y < top_y_left)
            // {
            //     top_x_left = hull[i].x;
            //     top_y_left = hull[i].y;
            //     top_num_left = i;
            // }
            if(sqrt(pow(hull[i].x - img.cols*7/8, 2) + pow(hull[i].y - 0, 2)) < sqrt(pow(top_x_right - img.cols*7/8, 2) + pow(top_y_right - 0, 2)))
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

		double daegaksun = sqrt(pow(bottom_x_left - top_x_right, 2) + pow(bottom_y_left - top_y_right, 2));
		double long_sin = 0;
		double fake_sin;

		for(int j=0; j < hull.size(); j++)
		{


			if((hull[j].y < bottom_y_left -20) && (hull[j].x < top_x_right - 20))
			{
				double sasun1 = sqrt(pow(hull[j].x - bottom_x_left, 2) + pow(hull[j].y - bottom_y_left, 2));
				double sasun2 = sqrt(pow(hull[j].x - top_x_right, 2) + pow(hull[j].y - top_y_right, 2));

				double theta = acos((pow(sasun1, 2) + pow(sasun2, 2) - pow(daegaksun, 2)) / (2*sasun1*sasun2));

				fake_sin = sasun1 * sin(theta);

				if(fake_sin > long_sin)
				{
					long_sin = fake_sin;

					top_x_left = hull[j].x;
					top_y_left = hull[j].y;
				}

			}
		}

        double mean_top_x = (double)((top_x_left + top_x_right) / 2.0);
        double mean_top_y = (double)((top_y_left + top_y_right) / 2.0);
		double mean_mid_x = (double)((top_x_left+top_x_right+bottom_x_left+bottom_x_right)/4.0);
		double mean_mid_y = (double)((top_y_left+top_y_right+bottom_y_left+bottom_y_right)/4.0);
        double mean_bottom_x = (double)((bottom_x_left + bottom_x_right) / 2.0);
        double mean_bottom_y = (double)((bottom_y_left + bottom_y_right) / 2.0);

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
			imshow("Left", drawing);
		}
		else
		{
			imshow("Right", drawing);
		}
		
		return array;
    }
    else
    {
        return array;
    }
}

/**
 * @brief 실제 erp의 GPS를 기준으로 X, Z값을 계산하는 함수
 * 
 * left_point와 right_point는 서로 대응되는 점을 넣어야 함
 * ex) {left_array[0],left_array[1]}를 넣으면 {right_array[0],right_array[1]}를 넣어야 함
 * 
 * @param left_point find_center 함수로 구한 좌측 카메라에서의 x,y값
 * @param right_point find_center 함수로 구한 우측 카메라에서의 x,y값
 * @param leftFrame 왼쪽화면 입력영상
 * @param rightFrame 오른쪽화면 입력영상
 * @param alpha 카메라가 고개를 숙인 각도 (처음 find_ball함수를 이용해 캘리를 하면서 구함)
 * @param beta 카메라가 틀어진 각도 (처음 find_ball함수를 이용해 캘리를 하면서 구함)
 * @return Point2d (실제 X거리값, 실제 Z거리값, cm단위임)
 */
Point2d StereoVision::find_XZ(Point2d left_point, Point2d right_point, Mat& leftFrame, Mat& rightFrame, float alpha, float beta)
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

	// cout << " realX : " << realX << "   realY : "<< realY << "     realZ : " << realZ << endl << endl;
	// cout << " distance : " << distance << endl;
	
	// cout << " 영점 조절 : " << realX << endl;
	
	//ERP 기준 좌표로 변환
	alpha = alpha * CV_PI / 180;
	beta = beta * CV_PI / 180;

	float fakeZ = realZ;
	float fakeY = realY;

	float theta = atan(fakeY/fakeZ) - alpha;
	realZ = sqrt(pow(fakeZ,2)+pow(fakeY,2))*cos(theta);
	realY = sqrt(pow(fakeZ,2)+pow(fakeY,2))*sin(theta);

	float realZ_copy = realZ;
	float realX_copy = realX;

	float gama = atan(realX/realZ) + beta;
	realZ = sqrt(pow(realZ_copy,2)+pow(realX_copy,2))*cos(gama);
	realX = sqrt(pow(realZ_copy,2)+pow(realX_copy,2))*sin(gama);
	
	// float angle = 0;
	// angle = atan(realX/realZ)*180/CV_PI;

	// cout << "realZ : " << realZ << "  realX : " << realX << endl <<endl;
	return {realZ, realX};
}

/**
 * @brief 그림자 문제를 해결하기 위해 적응형 이진화 함수를 사용해 색공간이 아닌 다른 방법으로 영상을 이진화함
 * 
 * @param src 입력영상 
 * @return Mat 이진화된 영상 
 */
Mat StereoVision::adaptiveThreshold_parking(Mat src)
{
	Mat image;
	Mat binary;

	image = src.clone();

	resize(image, image, Size(1280, 960));

	imshow("cap", image);

	cvtColor(image, binary, CV_BGR2GRAY);
	// namedWindow("dst");
	// createTrackbar("Block_Size", "dst", 0, 200, on_trackbar, (void*)&binary);
	// setTrackbarPos("Block_Size", "dst", 11);

	adaptiveThreshold(binary, binary, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 9, 5);

	Mat binary_inv;

	binary_inv = ~binary;

	// morphologyEx(binary, binary, MORPH_OPEN, Mat(), Point(-1, -1), 3);

	morphologyEx(binary_inv, binary_inv, MORPH_CLOSE, Mat(), Point(-1, -1), 3);

	resize(binary_inv, binary_inv, Size(640, 480));

	imshow("dst", binary_inv);

	return binary_inv;
}