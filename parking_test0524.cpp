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

Mat img(Size(640, 480), CV_8UC3, Scalar(0,0,0));

int lower_Y = 0, upper_Y = 40;

int lower_hue = 0, upper_hue = 255;
int lower_sat = 0, upper_sat = 255;
int lower_val = 0, upper_val = 255;

int main(int argc, char **argv)
{
    VideoCapture cap("/home/usera/catkin_ws/src/record_video/video/vision_parking/straight.mp4");
    // VideoCapture cap(2);

    ros::init(argc, argv, "parking");
	ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image_raw", 10);
    sensor_msgs::ImagePtr msg;
    ros::Rate loop_rate(50);

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

        // 이미지 자르기
        rectangle(frame1, Rect(0, 0, 640, 190), Scalar(0, 0, 0), -1);



        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame1).toImageMsg();
		pub.publish(msg);

        imshow("src", frame1);

        Mat src;
        cv::resize(frame1, src, cv::Size(640, 480), 0, 0);

       

        // 히스토그램 평활화

        Mat src_ycrcb;
        cv::cvtColor(src, src_ycrcb, COLOR_BGR2YCrCb);

        vector<Mat> ycrcb_planes;
        split(src_ycrcb, ycrcb_planes);

        equalizeHist(ycrcb_planes[0], ycrcb_planes[0]);

        Mat dst_ycrcb;
        merge(ycrcb_planes, dst_ycrcb);

        Mat dst;
        cvtColor(dst_ycrcb, dst, COLOR_YCrCb2BGR);

        

        // YUV색 공간 변경
        Mat src_YUV;
        cv::cvtColor(dst, src_YUV, COLOR_BGR2YUV);

        // 가우시안 블러
        // Mat gaussian;
        // GaussianBlur(src_YUV, gaussian, Size(), 1);

        frame2 = src_YUV.clone();
        // frame2 = src_YUV.clone();

        imshow("YUV", frame2);
        setMouseCallback("YUV", on_mouse2);

        Mat binary;
        inRange(frame2, Scalar(190, 122, 128), Scalar(255, 255, 255), mask);
        
        // 모폴로지 연산
        dilate(mask, mask, Mat(), Point(-1, -1), 1);
        erode(mask, mask, Mat(), Point(-1, -1), 1);

        dilate(mask, mask, Mat(), Point(-1, -1), 1);

        

        // erode(mask, mask, Mat(), Point(-1, -1), 1);
        // dilate(mask, mask, Mat(), Point(-1, -1), 1); 

        imshow("binary", mask);

        // namedWindow("mask", WINDOW_AUTOSIZE);
        // createTrackbar("Lower hue", "mask", &lower_hue, 255, on_hsv_change);
        // createTrackbar("Upper hue", "mask", &upper_hue, 255, on_hsv_change);
        // createTrackbar("Lower sat", "mask", &lower_sat, 255, on_hsv_change);
        // createTrackbar("Upper sat", "mask", &upper_sat, 255, on_hsv_change);
        // createTrackbar("Lower val", "mask", &lower_val, 255, on_hsv_change);
        // createTrackbar("Upper val", "mask", &upper_val, 255, on_hsv_change);
        // on_hsv_change(0, 0);

        // namedWindow("mask", WINDOW_AUTOSIZE);
        // createTrackbar("Lower Y", "mask", &lower_hue, 255, on_YUV_change);
        // createTrackbar("Upper Y", "mask", &upper_hue, 255, on_YUV_change);
        // createTrackbar("Lower U", "mask", &lower_sat, 255, on_YUV_change);
        // createTrackbar("Upper U", "mask", &upper_sat, 255, on_YUV_change);
        // createTrackbar("Lower V", "mask", &lower_val, 255, on_YUV_change);
        // createTrackbar("Upper V", "mask", &upper_val, 255, on_YUV_change);
        // on_YUV_change(0, 0);

        // 트랙바 안쓰고 inRange
        //inRange(frame2, Scalar(170, 120, 120), Scalar(255, 135, 145), mask);

        // imshow("mask", mask);

        // Mat mask_warp;
        // mask_warp = bird_eyes_view(mask);



        Mat mask2 = mask.clone();

        // contours
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(mask2, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

        Scalar color(rand() & 255, rand() & 255, rand() & 255);

        int lrgctridx = 0;
        int maxarea = 0;
        int nowarea = 0;
        for (int i = 0; i < contours.size(); i++)
        {
            double a = contourArea(contours[i]);

            // 모든 외각선의 사이즈 출력
            cout << "contour idx = " << i << " " << "size = " << a << endl;

            if (a > 14000 & a < 31000)
            {
                nowarea = a;
                if(a > maxarea)
                {
                    maxarea = a;
                    lrgctridx = i;
                }
                
            }
        }

        Mat drawing = Mat::zeros( mask2.size(), CV_8UC3 );

        // 모든 외각선 그리기
        // for(int idx = 0; idx >= 0; idx = hierarchy[idx][0])
        // {
        //     drawContours( drawing, contours, idx, color, 2, LINE_8, hierarchy);
        // }

        // 특정한 외각선만 그리기
        drawContours( drawing, contours, lrgctridx, color, 2, LINE_8, hierarchy);
        

        // 무게중심점
        Moments m = moments(contours[lrgctridx], true);
        Point p(m.m10/m.m00, m.m01/m.m00);

        cout << "centroid : "<< p << endl;
        circle(drawing, p, 10, Scalar(255, 255, 255), -1);
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
        return 0;

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