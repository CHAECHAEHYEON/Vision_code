#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include "std_msgs/Int32.h" // Int32형 메시지 주고받을 수 있게 함
#include <std_msgs/Float64.h> //Float64형 메시지 주고방르 수 있게 함
#include <sstream> //C++ 표준 라이브러리 일종
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h> // ros에서 opencv사용할 수 있게 함

using namespace cv;
using namespace std;

void on_mouse2(int event, int x, int y, int flags, void*);
void on_YUV_change(int, void*);
// void on_hsv_change(int, void*);
Mat bird_eyes_view(Mat img);

Mat frame1, frame2, mask, frame3;

int lower_Y = 0, upper_Y = 40;

int lower_hue = 0, upper_hue = 255;
int lower_sat = 0, upper_sat = 255;
int lower_val = 0, upper_val = 255;

int main(int argc, char **argv)
{
    VideoCapture cap("/home/park-kyeong-geun/parking_video/camera_right.mp4");

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    while(1)
    {
        
        if (!cap.isOpened())
        {
            cerr << "finish!\n" << endl;
        }

        waitKey();
        cap >> frame1;

        imshow("src", frame1);

        Mat src;
        cv::resize(frame1, src, cv::Size(640, 480), 0, 0);

        Mat src_YUV;

        // YUV색 공간 변경
        cv::cvtColor(src, src_YUV, COLOR_BGR2YUV);

        // 가우시안 블러
        Mat gaussian;
        GaussianBlur(src_YUV, gaussian, Size(), 3);

        frame2 = gaussian.clone();

        imshow("YUV", frame2);
        setMouseCallback("YUV", on_mouse2);

        // namedWindow("mask", WINDOW_AUTOSIZE);
        // createTrackbar("Lower hue", "mask", &lower_hue, 255, on_hsv_change);
        // createTrackbar("Upper hue", "mask", &upper_hue, 255, on_hsv_change);
        // createTrackbar("Lower sat", "mask", &lower_sat, 255, on_hsv_change);
        // createTrackbar("Upper sat", "mask", &upper_sat, 255, on_hsv_change);
        // createTrackbar("Lower val", "mask", &lower_val, 255, on_hsv_change);
        // createTrackbar("Upper val", "mask", &upper_val, 255, on_hsv_change);
        // on_hsv_change(0, 0);

        namedWindow("mask", WINDOW_AUTOSIZE);
        createTrackbar("Lower Y", "mask", &lower_hue, 255, on_YUV_change);
        createTrackbar("Upper Y", "mask", &upper_hue, 255, on_YUV_change);
        createTrackbar("Lower U", "mask", &lower_sat, 255, on_YUV_change);
        createTrackbar("Upper U", "mask", &upper_sat, 255, on_YUV_change);
        createTrackbar("Lower V", "mask", &lower_val, 255, on_YUV_change);
        createTrackbar("Upper V", "mask", &upper_val, 255, on_YUV_change);
        on_YUV_change(0, 0);

        // 트랙바 안쓰고 inRange
        //inRange(frame2, Scalar(170, 120, 120), Scalar(255, 135, 145), mask);

        // imshow("mask", mask);

        // Mat mask_warp;
        // mask_warp = bird_eyes_view(mask);



        Mat mask2 = mask.clone();

        // contours
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(mask2, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        Scalar color(rand() & 255, rand() & 255, rand() & 255);

        Mat drawing = Mat::zeros( mask2.size(), CV_8UC3 );
        for(int idx = 0; idx >= 0; idx = hierarchy[idx][0])
        {
            drawContours( drawing, contours, idx, color, 2, LINE_8, hierarchy);
        }
        imshow( "contour", drawing );
        
  
        // LineSegmentDetector로 라인 측정
        // 트랙바로 조정하면 안됨
        // cv::Ptr<cv::LineSegmentDetector> det;
        // det = cv::createLineSegmentDetector(); 
 
 
        // // Mat line_detect;
        // // cvtColor(mask2, line_detect, COLOR_BGR2GRAY);

        // cv::Mat lines;
        // det->detect(mask2, lines);

        // Mat dst(480, 640, CV_8UC1);
        // cvtColor(mask2, dst, COLOR_GRAY2BGR);
        // det->drawSegments(dst, lines);

        // cv::imshow("input", dst);   


        // Cany edge and Houph line

        // Mat edge;
        // Canny(mask_warp, edge, 50, 150);

        // vector<Vec4i> lines;
        // HoughLinesP(edge, lines, 1, CV_PI / 180, 1, 10, 5);

        // Mat dst;
        // cvtColor(edge, dst, COLOR_GRAY2BGR);
        // for (Vec4i l : lines)
        // {
        //     line(dst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2, LINE_AA);
        // }

        // imshow("dst", dst);
    }
}


void on_mouse2(int event, int x, int y, int flags, void*)
{
	switch (event)
	{
		case EVENT_LBUTTONDBLCLK:
		vector<Mat> hsv;
		cerr << "[Y, U, V] : " << frame2.at<Vec3b>(y, x) << "\n" << endl;
		// cerr << "v : " << hsv[2] << "\n" << endl;
		break;
	}
}

// void on_mouse2(int event, int x, int y, int flags, void*)
// {
// 	switch (event)
// 	{
// 		case EVENT_LBUTTONDBLCLK:
// 		vector<Mat> hsv;
// 		cerr << "[H, S, V] : " << frame2.at<Vec3b>(y, x) << "\n" << endl;
// 		// cerr << "v : " << hsv[2] << "\n" << endl;
// 		break;
// 	}
// }

// void on_hsv_change(int, void*)
// {
//    Scalar lowerb(lower_hue, lower_sat, lower_val);
//    Scalar upperb(upper_hue, upper_sat, upper_val);
//    inRange(frame2, lowerb, upperb, mask);

// //    imshow("mask", mask);
// }

void on_YUV_change(int, void*)
{
   Scalar lowerb(lower_hue, lower_sat, lower_val);
   Scalar upperb(upper_hue, upper_sat, upper_val);
   inRange(frame2, lowerb, upperb, mask);

   imshow("mask", mask);
}
