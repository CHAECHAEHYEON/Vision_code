#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <vector>

#include "StereoVision.h"

using namespace cv;
using namespace std;
Mat img_color;
Mat img_color_2;
int H, S, V;

float baseline = 23;
float focal_pixels = 800; // size(1280, 720) 일때
float alpha = 25; //카메라가 고개 숙인 각도
float beta = 45; // 카메라가 erp 헤딩으로부터 꺾인 각도

void mouse_callback(int event, int x, int y, int flags, void *param);
void mouse_callback_2(int event, int x, int y, int flags, void *param);
void on_mouse(int event, int x, int y, int flags, void *);
Point ptOld1;

int *mass_center1(Mat img, int arr[]);
int *mass_center2(Mat img, int arr[]);

float camera_values();



int main() {


    VideoCapture capLeft(2);
    VideoCapture capRight(4);
    
    // VideoCapture capLeft("/home/korus/catkin_ws/src/record_video/parking/parking5_0703.mp4");
    // VideoCapture capRight("/home/korus/catkin_ws/src/record_video/parking/parking5_0703.mp4");

    Mat leftFrame, rightFrame;
    Mat leftFrame_copy, rightFrame_copy;
    // capLeft.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	// capLeft.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    // capRight.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	// capRight.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    capLeft.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	capLeft.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    capRight.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	capRight.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    StereoVision stereovision(baseline, focal_pixels);

    if (!capLeft.isOpened()) {
        cout << "Cannot Open Left Camera" << endl;
        return -1;
    }

    if (!capRight.isOpened()) {
        cout << "Cannot Open Right Camera" << endl;
        return -1;
    }

    Mat leftMask, rightMask;
    Mat leftResFrame, rightResFrame;

    // Point leftCircle, rightCircle;
    string distance;
    float ballDistance = 0;

    while (true) {
        

        capLeft.read(leftFrame);
        capRight.read(rightFrame);
        
        // Calibration of the frames (640,480)
        // leftFrame = stereovision.undistortFrame(leftFrame);
        // rightFrame = stereovision.undistortFrame(rightFrame);
        
        
        // Applying HSV-filter
        leftMask = stereovision.add_HSV_filter(leftFrame, 0);
        rightMask = stereovision.add_HSV_filter(rightFrame, 1);

        
        // Frames after applyting HSV-filter mask
        bitwise_and(leftFrame, leftFrame, leftResFrame, leftMask);
        bitwise_and(rightFrame, rightFrame, rightResFrame, rightMask);

        // Detect Circles - Hough Transforms can be used aswell or some neural network to do object detection
        // leftCircle = stereovision.find_ball(leftMask, leftMask);
        // rightCircle = stereovision.find_ball(rightMask, rightMask);
        int array[] = {0,0};
        int *leftCircle = mass_center1(leftMask,array);
        int *rightCircle = mass_center2(rightMask,array);
        
        cout << "array = [ ";
        for (int i = 0; i < 2; ++i) {
            cout << leftCircle[i] << ", ";
        }
        cout << "]" << endl;

    //    // If no ball is detected in one of the cameras - show the text "tracking lost"
    //     if (!leftCircle[0] || !rightCircle[0]) {
    //         putText(leftMask, "Tracking Lost!", { 75, 50 }, FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2);
    //         putText(rightMask, "Tracking Lost!", { 75, 75 }, FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2);
    //     } else {

    //         // Vector of all depths in case of several balls detected.
    //         // All formulas used to find depth is in the video presentation
    //         ballDistance = stereovision.find_depth(leftCircle, rightCircle, leftFrame, rightFrame, alpha, beta);

    //         putText(leftMask, "Tracking!", { 75, 50 }, FONT_HERSHEY_SIMPLEX, 0.7, (50,170,50), 2);
    //         putText(rightMask, "Tracking!", { 75, 75 }, FONT_HERSHEY_SIMPLEX, 0.7, (50,170,50), 2);

    //         distance = to_string(ballDistance);
    //         putText(leftMask, distance + " CM", Point(400,360), FONT_HERSHEY_SIMPLEX, 2, Scalar(50,170,50),2);
    //         putText(rightMask, distance + " CM", Point(400,360), FONT_HERSHEY_SIMPLEX, 2, Scalar(50,170,50),2);

    //         // Multiply computer value with 205.8 to get real - life depth in[cm]. The factor was found manually. 
    //     }

        
        //left camera, right camera 한 평면 위에 놓는 작업

        // leftFrame_copy = leftFrame.clone();
        // rightFrame_copy = rightFrame.clone();

        // rectangle(leftFrame_copy, Rect(Point(635,0),Point(645,720)),Scalar(0,255,0),2,4,0);
        // rectangle(leftFrame_copy, Rect(Point(0,680),Point(1280,690)),Scalar(0,255,0),2,4,0);
        // rectangle(leftFrame_copy, Rect(Point(0,640),Point(1280,650)),Scalar(0,255,0),2,4,0);
        // rectangle(leftFrame_copy, Rect(Point(0,600),Point(1280,610)),Scalar(0,255,0),2,4,0);
        // rectangle(leftFrame_copy, Rect(Point(0,560),Point(1280,570)),Scalar(0,255,0),2,4,0);
        // rectangle(leftFrame_copy, Rect(Point(0,520),Point(1280,530)),Scalar(0,255,0),2,4,0);
        // rectangle(leftFrame_copy, Rect(Point(0,480),Point(1280,490)),Scalar(0,255,0),2,4,0);
        // rectangle(leftFrame_copy, Rect(Point(0,360),Point(1280,370)),Scalar(0,255,0),2,4,0);

        // rectangle(rightFrame_copy, Rect(Point(635,0),Point(645,720)),Scalar(0,255,0),2,4,0);
        // rectangle(rightFrame_copy, Rect(Point(0,680),Point(1280,690)),Scalar(0,255,0),2,4,0);
        // rectangle(rightFrame_copy, Rect(Point(0,640),Point(1280,650)),Scalar(0,255,0),2,4,0);
        // rectangle(rightFrame_copy, Rect(Point(0,600),Point(1280,610)),Scalar(0,255,0),2,4,0);
        // rectangle(rightFrame_copy, Rect(Point(0,560),Point(1280,570)),Scalar(0,255,0),2,4,0);
        // rectangle(rightFrame_copy, Rect(Point(0,520),Point(1280,530)),Scalar(0,255,0),2,4,0);
        // rectangle(rightFrame_copy, Rect(Point(0,480),Point(1280,490)),Scalar(0,255,0),2,4,0);
        // rectangle(rightFrame_copy, Rect(Point(0,360),Point(1280,370)),Scalar(0,255,0),2,4,0);

        // imshow("Left Frame line", leftFrame_copy);
        // imshow("right Frame line", rightFrame_copy);


        imshow("Left Frame", leftFrame);
        imshow("Right Frame", rightFrame);
        imshow("Left Mask", leftMask);
        imshow("Right Mask", rightMask);
   
   
        // img_color = leftFrame.clone();
        // img_color_2 = rightFrame.clone();
        setMouseCallback("Left Frame", mouse_callback);
        setMouseCallback("Right Frame", mouse_callback_2);
        // setMouseCallback("Left Frame",on_mouse);
        // setMouseCallback("Right Frame",on_mouse);
        // waitKey();
        // Hit "q" to close the window
        if ((waitKey(1) & 0xFF) == 'q') {
            break;
        }
    }

    // Release and destroy all windows before termination
    capLeft.release();
    capRight.release();

    destroyAllWindows();

    return 0;
}

void mouse_callback(int event, int x, int y, int flags, void *param)
{
	if (event == EVENT_LBUTTONDBLCLK)
	{
		Vec3b color_pixel = img_color.at<Vec3b>(y, x);

		Mat hsv_color = Mat(1, 1, CV_8UC3, color_pixel);


		H = hsv_color.at<Vec3b>(0, 0)[0];
		S = hsv_color.at<Vec3b>(0, 0)[1];
		V = hsv_color.at<Vec3b>(0, 0)[2];

		cout << "left H= " << H << endl;
		cout << "left S= " << S << endl;
		cout << "left V = " << V << "\n"
			 << endl;

	}
}
void mouse_callback_2(int event, int x, int y, int flags, void *param)
{
	if (event == EVENT_LBUTTONDBLCLK)
	{
		Vec3b color_pixel = img_color_2.at<Vec3b>(y, x);

		Mat hsv_color = Mat(1, 1, CV_8UC3, color_pixel);


		H = hsv_color.at<Vec3b>(0, 0)[0];
		S = hsv_color.at<Vec3b>(0, 0)[1];
		V = hsv_color.at<Vec3b>(0, 0)[2];

		cout << "right H= " << H << endl;
		cout << "right S= " << S << endl;
		cout << "right V = " << V << "\n"
			 << endl;

	}
}

int *mass_center1(Mat img, int arr[])
{
    Mat mask2 = img.clone();

    // imshow("R_mask", mask2);

    // contours
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask2, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

    Scalar color(rand() & 255, rand() & 255, rand() & 255);

    int lrgctridx = 0;
    int minarea = 1000000;
    int nowarea = 0;

    
    for (int i = 0; i < contours.size(); i++)
    {
        double a = contourArea(contours[i]);

        // 모든 외각선의 사이즈 출력
        // cout << "contour idx = " << i << " " << "size = " << a << endl;

        if (a > 1000 & a < 3000000)
        {
            nowarea = a;
            if(a < minarea)
            {
                minarea = a;
                lrgctridx = i;
            }
            
        }
    }

    if(contours.size() > 0)
    {

        Mat drawing = Mat::zeros( mask2.size(), CV_8UC3 );

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
            if(hull[i].y <= top_y_left)
            {
                top_x_left = hull[i].x;
                top_y_left = hull[i].y;
                top_num_left = i;
            }
            if(sqrt(pow(hull[i].x - 0, 2) + pow(hull[i].y - img.rows, 2)) <= sqrt(pow(bottom_x_left - 0, 2) + pow(bottom_y_left - img.rows, 2)))
            {
                bottom_x_left = hull[i].x;
                bottom_y_left = hull[i].y;
                bottom_num_left = i;
            }
        }

        for(int i = 0; i < hull.size(); i++)
        {
            if(sqrt(pow(hull[i].x - img.cols, 2) + pow(hull[i].y - 0, 2)) <= sqrt(pow(top_x_right - img.cols, 2) + pow(top_y_right - 0, 2)))
            {
                top_x_right = hull[i].x;
                top_y_right = hull[i].y;
                top_num_right = i;
            }
            if(sqrt(pow(hull[i].x - img.cols, 2) + pow(hull[i].y - img.rows, 2)) <= sqrt(pow(top_x_right - img.cols, 2) + pow(top_y_right - img.rows, 2)))
            {
                bottom_x_right = hull[i].x;
                bottom_y_right = hull[i].y;
                bottom_num_right = i;
            }
        }

        // 무게중심점
        Moments m = moments(contours[lrgctridx], true);
        Point p(m.m10/m.m00, m.m01/m.m00);

        int mean_top_x = int((top_x_left + top_x_right) / 2);
        int mean_top_y = int((top_y_left + top_y_right) / 2);
        int mean_bottom_x = int((bottom_x_left + bottom_x_right) / 2);
        int mean_bottom_y = int((bottom_y_left + bottom_y_right) / 2);

        circle (drawing, p, 3, color, -1);
        circle(drawing, Point(mean_top_x, mean_top_y), 10, Scalar(0, 255, 0), -1);
        circle(drawing, Point(mean_bottom_x, mean_bottom_y), 10, Scalar(0, 255, 0), -1);

        imshow("L_drawing", drawing);

        // arr[] = {mean_bottom_x,mean_bottom_y,p.x,p.y,mean_top_x,mean_top_y};
        arr[0] = p.x;
        arr[1] = p.y;
        return arr;
    }
    else
    {
        // return {0,0,0,0,0,0};
        arr[0] = 0;
        arr[1] = 0;
        return arr;
    }

}

int *mass_center2(Mat img, int arr[])
{


    Mat mask2 = img.clone();
    int arr[1][2];

    // imshow("L_mask", mask2);

    // contours
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask2, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

    Scalar color(rand() & 255, rand() & 255, rand() & 255);

    int lrgctridx = 0;
    int minarea = 1000000;
    int nowarea = 0;
    for (int i = 0; i < contours.size(); i++)
    {
        double a = contourArea(contours[i]);

        // 모든 외각선의 사이즈 출력
        // cout << "contour idx = " << i << " " << "size = " << a << endl;

        if (a > 1000 & a < 3000000)
        {
            nowarea = a;
            if(a < minarea)
            {
                minarea = a;
                lrgctridx = i;
            }
            
        }
    }

    if(contours.size() > 0)
    {

        Mat drawing = Mat::zeros( mask2.size(), CV_8UC3 );

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
            if(hull[i].y <= top_y_left)
            {
                top_x_left = hull[i].x;
                top_y_left = hull[i].y;
                top_num_left = i;
            }
            if(sqrt(pow(hull[i].x - 0, 2) + pow(hull[i].y - img.rows, 2)) <= sqrt(pow(bottom_x_left - 0, 2) + pow(bottom_y_left - img.rows, 2)))
            {
                bottom_x_left = hull[i].x;
                bottom_y_left = hull[i].y;
                bottom_num_left = i;
            }
        }

        for(int i = 0; i < hull.size(); i++)
        {
            if(sqrt(pow(hull[i].x - img.cols, 2) + pow(hull[i].y - 0, 2)) <= sqrt(pow(top_x_right - img.cols, 2) + pow(top_y_right - 0, 2)))
            {
                top_x_right = hull[i].x;
                top_y_right = hull[i].y;
                top_num_right = i;
            }
            if(sqrt(pow(hull[i].x - img.cols, 2) + pow(hull[i].y - img.rows, 2)) <= sqrt(pow(top_x_right - img.cols, 2) + pow(top_y_right - img.rows, 2)))
            {
                bottom_x_right = hull[i].x;
                bottom_y_right = hull[i].y;
                bottom_num_right = i;
            }
        }
        // 무게중심점
        Moments m = moments(contours[lrgctridx], true);
        Point p(m.m10/m.m00, m.m01/m.m00);

        int mean_top_x = int((top_x_left + top_x_right) / 2);
        int mean_top_y = int((top_y_left + top_y_right) / 2);
        int mean_bottom_x = int((bottom_x_left + bottom_x_right) / 2);
        int mean_bottom_y = int((bottom_y_left + bottom_y_right) / 2);

        circle (drawing, p, 3, color, -1);
        circle(drawing, Point(mean_top_x, mean_top_y), 10, Scalar(0, 255, 0), -1);
        circle(drawing, Point(mean_bottom_x, mean_bottom_y), 10, Scalar(0, 255, 0), -1);
        


        imshow("R_drawing", drawing);
        // arr[] = {mean_bottom_x,mean_bottom_y,p.x,p.y,mean_top_x,mean_top_y};
        arr[0] = p.x;
        arr[1] = p.y;
        return arr;
    }
    else
    {
        // return {0,0,0,0,0,0};
        return {0,0};
    }

}

float camera_values()
{
    cv::Mat K(3,3,CV_64F);

    K = (Mat_<_Float64>(3, 3) << 538.39128993937,   0.,   308.049009633327, 0.,  539.777910345317,  248.065244763797, 0., 0., 1.);

    cout << "K : " << K << endl;


    cv::Size imageSize(640,480);
    double apertureWidth = 0;
    double apertureHeight = 0;
    double fieldOfViewX;
    double fieldOfViewY;
    double focalLength2;
    cv::Point2d principalPoint;
    double aspectRatio;
    cv::calibrationMatrixValues(K, imageSize, apertureWidth, apertureHeight, fieldOfViewX, fieldOfViewY, focalLength2, principalPoint, aspectRatio);

    
    cout << fieldOfViewX << endl;

    return (float)focalLength2;
}

void on_mouse(int event, int x, int y, int flags, void *)
{
	switch (event)
	{
	case EVENT_LBUTTONDOWN:
		ptOld1 = Point(x, y);
		cout << "EVENT_LBUTTONDOWN: " << x << ", " << y << endl;
		break;
	case EVENT_LBUTTONUP:
		cout << "EVENT_LBUTTONUP: " << x << ", " << y << endl;
		break;
	}
}