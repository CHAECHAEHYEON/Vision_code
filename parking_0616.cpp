#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <iostream>
#include <stdio.h>
#include <math.h> 
#include <ros/ros.h>
#include "std_msgs/Int32.h" // Int32형 메시지 주고받을 수 있게 함
#include <std_msgs/Float64.h> //Float64형 메시지 주고방르 수 있게 함
#include <sstream> //C++ 표준 라이브러리 일종
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h> // ros에서 opencv사용할 수 있게 함

#define SSTR( x ) static_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x) ).str()

#define PI 3.141592

using namespace cv;
using namespace std;

void on_mouse1(int event, int x, int y, int flags, void*);
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

Point ptOld1;

float min_theta = 90;




int main(int argc, char **argv)
{
    VideoCapture cap("/home/kroad/catkin_ws/src/parking/src/camera_right2.mp4");
    // VideoCapture cap(2);

    ros::init(argc, argv, "parking");
	ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image_raw", 10);
    sensor_msgs::ImagePtr msg;
    ros::Rate loop_rate(50);

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    string trackerTypes[7] = {"BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW", "MOSSE", "CSRT"};

    string trackerType = trackerTypes[0];

    Ptr<Tracker> tracker;

    if (trackerType == "BOOSTING")
        tracker = TrackerBoosting::create();
    if (trackerType == "MIL")
        tracker = TrackerMIL::create();
    if (trackerType == "KCF")
        tracker = TrackerKCF::create();
    if (trackerType == "TLD")
        tracker = TrackerTLD::create();
    if (trackerType == "MEDIANFLOW")
        tracker = TrackerMedianFlow::create();
    if (trackerType == "MOSSE")
        tracker = TrackerMOSSE::create();
    if (trackerType == "CSRT")
        tracker = TrackerCSRT::create();

    Mat frame;

    bool ok = cap.read(frame);

    Rect2d bbox(Point(332, 193), Point(406, 238));
    // Rect2d bbox(Point(191, 140), Point(259, 175));

    rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );

    imshow("Tracking", frame);
    tracker->init(frame, bbox);


    while(ros::ok())
    {    
        
        if (!cap.isOpened())
        {
            cerr << "finish!\n" << endl;
        }

        cap >> frame;

        double timer = (double)getTickCount();

        bool ok = tracker->update(frame, bbox);

        float fps = getTickFrequency() / ((double)getTickCount() - timer);

        // 마스크용 영상
        Mat mask = Mat::zeros(480, 640, CV_8UC1);

        // line detect video
        Mat line_detect = Mat::zeros(480, 640, CV_8UC3);

        if (ok)
        {
            rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1);
            rectangle(mask, bbox, Scalar( 255, 255, 255), -1);
            frame.copyTo(line_detect, mask);

        }
        else
        {
            putText(frame, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);
        }

        putText(frame, trackerType + " Tracker", Point(100,20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50),2);

        putText(frame, "FPS : " + SSTR(int(fps)), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);

        imshow("Tracking", frame);
        // imshow("MASK", mask);

        Mat src_HLS;
        cvtColor(line_detect, src_HLS, COLOR_BGR2HLS);
        inRange(src_HLS, Scalar(0, 190, 0), Scalar(255, 255, 255), src_HLS);

        dilate(src_HLS, src_HLS, Mat());
        erode(src_HLS, src_HLS, Mat());
        dilate(src_HLS, src_HLS, Mat());

        // imshow("HLS", src_HLS);

        // 소벨 마스크
        Mat dx, dy;
        Sobel(src_HLS, dx, CV_32FC1, 1, 0);
        Sobel(src_HLS, dy, CV_32FC1, 0, 1);

        Mat fmag, mag;
        magnitude(dx, dy, fmag);
        fmag.convertTo(mag, CV_8UC1);

        Mat edge = mag > 150;

        imshow("edge", edge);

        // 캐니엣지
        // Mat edge;
        // Canny(src_HLS, edge, 50, 150);

        //허프라인으로 선 감지

        // 방법1
        // Mat line_detect_G;
	    // cvtColor(frame, line_detect_G, COLOR_BGR2GRAY);

        // vector<Vec2f> lines;
        // HoughLines(edge, lines, 1, CV_PI / 180, 40);

        // Mat dst;
        // cvtColor(edge, dst, COLOR_GRAY2BGR);

        // for(size_t i = 0; i < lines.size(); i++)
        // {
        //     float r = lines[i][0], t = lines[i][1];
        //     double cos_t = cos(t), sin_t = sin(t);
        //     double x0 = r * cos_t, y0 = r * sin_t;
        //     double alpha = 1000;

        //     Point pt1(cvRound(x0 + alpha * (-sin_t)), cvRound(y0 + alpha * cos_t));
        //     Point pt2(cvRound(x0 - alpha * (-sin_t)), cvRound(y0 - alpha * cos_t));
        //     line(dst, pt1, pt2, Scalar(0, 0, 255), 2, LINE_AA);
        // }

        // 방법 2
	    vector<Vec4i> lines;    
    	HoughLinesP(edge, lines, 1, CV_PI / 180, 5, 20, 0);

        cvtColor(edge, edge, COLOR_GRAY2BGR);

    	
    	Mat dst(Size(640, 480), CV_8UC3);

	    for (Vec4i l : lines)
    	{

	    	// line(edge, Point(l[0],l[1]),Point(l[2],l[3]),Scalar(0,0,255), 2,LINE_AA);
		    //cout << "lines.size: " << lines.size() << endl;
		   
            float dx = l[2] - l[0], dy = l[3] - l[1];
		    float k = dy / dx;
		    float radian = atan2(dy, dx);
            float theta = (radian * 180) / CV_PI;
            float d = sqrt(pow(dx, 2) + pow(dy, 2));
            float y1 = l[1] - 10;
            float y2 = l[3] + 10;
            float x0 = (l[0] + l[2]) / 2;
            float y0 = (l[1] + l[3]) / 2;
           
            //cout << "theta: " << theta << endl;

            // if (theta < 89 && theta>88)
            // {
            // 	cout << "stop!!!!!!!!!!!!!!!!" << theta << endl;
            // 	isStop = 1;
            // }
            // cout<<"theta: "<<theta<<endl;
            if (theta > -90 && theta < -70)
            {
                theta = -theta;
                line(edge, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 10, LINE_AA);
                circle(edge, Point(l[0], l[1]), 3, Scalar(255, 0, 0), -1);
                if(theta < min_theta) min_theta = theta;
                // cout << "theta_original: " << theta << "\n" << endl;
            }  
            else if (theta > 30 && theta < 100)
            {
                line(edge, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 10, LINE_AA);
                circle(edge, Point(l[0], l[1]), 3, Scalar(255, 0, 0), -1);
                if(theta < min_theta) min_theta = theta;
                // cout << "theta_original: " << theta << "\n" << endl;
            }
            else
            {
                theta = 0;
            }
            //     //cout << "Point(l[0], l[1]): " << Point(l[0], l[1]) << " Point(l[2], l[3]): " << Point(l[2], l[3]) << endl;
            // line(dst, Point(l[0], l[1]), Point(l[20;], l[3]), Scalar(255, 255, 255), 15, LINE_AA);
            cout << "theta_original : " << theta << "\n" << endl;
            cout << "min_theta : " << min_theta << "\n" << endl;
            // else if (isStop == 0)
            // {
            // 	cout << "nothing" << endl;
            // }
        }
        imshow("line_detect", line_detect);
        imshow("dst", edge);
        
        Mat final;
        final = edge + frame;

        putText(final, "theta_live : " + SSTR(float(min_theta)), Point(300,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);

        imshow("final", final);

        waitKey();

        ros::spinOnce();
		loop_rate.sleep();
        /*
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

        // imshow("dst", dst);*/

    }
        return 0;

}

void on_mouse1(int event, int x, int y, int flags, void*)
{
	switch (event)
	{
		case EVENT_LBUTTONDBLCLK:
            ptOld1 = Point(x, y);
            cout << x << ", " << y << endl;
            break;
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