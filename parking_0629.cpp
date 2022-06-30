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

#include <vector>

#define SSTR( x ) static_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x) ).str()

#define PI 3.141592

using namespace cv;
using namespace cv::ml;
using namespace std;

void parking_cb(const std_msgs::Bool::ConstPtr &msgs);
void imageCallback(const sensor_msgs::ImageConstPtr &msg);
void on_mouse1(int event, int x, int y, int flags, void*);
void on_mouse2(int event, int x, int y, int flags, void*);
void on_YUV_change(int, void*);

Mat bird_eyes_view(Mat img);

Mat frame1, frame2, frame3, mask;

Mat img(Size(640, 480), CV_8UC3, Scalar(0,0,0));

int lower_Y = 0, upper_Y = 40;

int lower_hue = 0, upper_hue = 255;
int lower_sat = 0, upper_sat = 255;
int lower_val = 0, upper_val = 255;

Point ptOld1;

float min_theta = 90;
float final_theta = 90;

int isStop = 10;
int right_line = 0;     // 기존의 SecondLevel
bool parking1 = false;
float encoder = 3;
bool enc_fail = false;
bool callback = false;

// ros::Publisher parking_pub;
ros::Subscriber Parking_level1_sub;		// 라이다
ros::Publisher parking_level3_pub;		// 1번선 각도
ros::Publisher parking_level4_pub;		// 2번선 각도
ros::Publisher parking_level4_pub1;		// 오른쪽 or 왼쪽 틀기
ros::Publisher stopline_pub;			// 2번선까지의 거리
ros::Publisher encoder_pub;				// 라이다가 신호주면 엔코더값 기록, 정지
ros::Subscriber Parking_encoder_fail_sub;		// 2번선 따라 멈추고 조향, 검출 안되면 엔코더 fail이 나서 멈추기


void parking_cb(const std_msgs::Bool::ConstPtr &msgs)
{
	parking1 = msgs->data;
}

void enc_fail_cb(const std_msgs::Bool::ConstPtr &msgs)
{
	enc_fail = msgs->data;
}



int main(int argc, char **argv)
{
    VideoCapture cap("/home/chaehyeon/catkin_ws/src/camera_right2.mp4");
    VideoCapture cap1(4);

    // VideoCapture cap(6);    // 우측 카메라
    // VideoCapture cap1(4);   // 정지선 카메라

    ros::init(argc, argv, "parking_publisher");
	ros::NodeHandle nh;

    
    Parking_level1_sub = nh.subscribe("/Lidar_Stop", 10, parking_cb); //parkingstop
	Parking_encoder_fail_sub = nh.subscribe("enc_fail", 10, enc_fail_cb);
	parking_level3_pub = nh.advertise<std_msgs::Float64>("ParkingAngle_RIGHT", 10);
	parking_level4_pub = nh.advertise<std_msgs::Float64>("ParkingAngle_FRONT", 10);
	parking_level4_pub1 = nh.advertise<std_msgs::Float64>("Plus_or_Minus", 10);
	stopline_pub = nh.advertise<std_msgs::Float64>("stopline_parking", 100);
	encoder_pub = nh.advertise<std_msgs::Float64>("encoder_mode", 100);
	image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_image = it.subscribe("/camera/stopline/image_raw", 100, imageCallback);
    // image_transport::Publisher pub = it.advertise("camera/image_raw", 10);
    // sensor_msgs::ImagePtr msg;
    ros::Rate loop_rate(50);
    printf("Waiting for ---/camera/stopline/image_raw---\n");

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



    while(ros::ok())
    {    
        if (!cap.isOpened())
        {
            cerr << "finish!\n" << endl;
        }

        cap >> frame; // side camera
        cap1 >> frame_for_front_camara; //front camara

        Mat frame;

        bool ok = cap.read(frame);

        Rect2d bbox(Point(332, 193), Point(406, 238));
        // Rect2d bbox(Point(191, 140), Point(259, 175));

        rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );

        imshow("Tracking", frame);
        tracker->init(frame, bbox);

        int count = 10;

        double timer = (double)getTickCount();

        bool ok = tracker->update(frame, bbox);

        float fps = getTickFrequency() / ((double)getTickCount() - timer);

        // 마스크용 영상
        Mat mask = Mat::zeros(480, 640, CV_8UC1);

        // line detect video
        Mat line_detect = Mat::zeros(480, 640, CV_8UC3);

        Mat final = frame.clone();

        Mat frame_part;

        if (ok)
        {
            rectangle(final, bbox, Scalar( 255, 0, 0 ), 2, 1);
            rectangle(mask, bbox, Scalar( 255, 255, 255), -1);
            frame_part = frame(bbox);
            frame.copyTo(line_detect, mask);

        }
        else
        {
            putText(final, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);
        }

        putText(final, trackerType + " Tracker", Point(100,20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50),2);

        putText(final, "FPS : " + SSTR(int(fps)), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);

        imshow("Tracking", frame);
        // imshow("frame_part", frame_part);

        int cols = frame_part.cols;
        int rows = frame_part.rows;

        Mat big_frame_part;
        resize(frame_part, big_frame_part, Size(cols*15, rows*15), 0, 0, INTER_CUBIC);
        imshow("big_frame_part", big_frame_part);

        // imshow("MASK", mask);

        // HLS 색공간으로 이진화
        Mat src_HLS;
        cvtColor(big_frame_part, src_HLS, COLOR_BGR2HLS);
        inRange(src_HLS, Scalar(0, 190, 0), Scalar(255, 255, 255), src_HLS);

        // 이진화 모폴로지 연산
        dilate(src_HLS, src_HLS, Mat());
        erode(src_HLS, src_HLS, Mat());
        dilate(src_HLS, src_HLS, Mat());

        // 흑백영상으로 바꿈
        Mat line_detect_G;
        cvtColor(line_detect, line_detect_G, COLOR_BGR2GRAY);

        imshow("HLS", src_HLS);

        // 소벨 마스크
        Mat dx, dy;
        Sobel(src_HLS, dx, CV_32FC1, 1, 0);
        Sobel(src_HLS, dy, CV_32FC1, 0, 1);

        Mat fmag, mag;
        magnitude(dx, dy, fmag);
        fmag.convertTo(mag, CV_8UC1);

        Mat edge = mag > 150;

        imshow("edge", edge);
        std_msgs::Float64 encoder_mode_msg;

        if(parking1 == true)
        {
            encoder = 2;	// 2번이 기록 시작
			start_level_4 = right_level(edge);
            // 최종 영상에 허프라인 결과 합성
            Mat final_roi = final(bbox);
            addWeighted(final_roi, 0.3, edge_resize, 1, 0, final_roi);
            putText(final, "theta_live : " + SSTR(float(final_theta)), Point(300,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);
            imshow("final", final);

            if(right_line = 1)
            {
                start_level4 = true;
				encoder = 2;
				cout << "stop level3!!!!!!!!!!!!!!!!!!!" << endl;
            }

        else if (start_level4 == true)
        {
            encoder = 2;
			cout << "44444444444444444444444" << endl;
			frame3 = preprocessing_forth_level(img_resize6); // img_resize6 정지선으로부터 받아온 영상. 정면캠
        }
        }

        waitKey(1);

        ros::spinOnce();
		loop_rate.sleep();

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


void on_YUV_change(int, void*)
{
   Scalar lowerb(lower_hue, lower_sat, lower_val);
   Scalar upperb(upper_hue, upper_sat, upper_val);
   inRange(frame2, lowerb, upperb, mask);

   imshow("mask", mask);
}

Mat right_level(Mat img)    // 기존의 preprocessing_second_level, 오른쪽 라인 검출
{
        // contours
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(src_HLS, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
        Scalar color(rand() & 255, rand() & 255, rand() & 255);
 

        vector< vector<Point> > hull(contours.size());

        for(int i = 0; i < contours.size(); i++)
            convexHull(Mat(contours[i]), hull[i], false);

        Mat drawing = Mat::zeros( edge.size(), CV_8UC3 );
        // 모든 외각선 그리기
        for(int idx = 0; idx >= 0; idx = hierarchy[idx][0])
        {
            // drawContours( drawing, contours, idx, color, 2, LINE_8, hierarchy);
            drawContours(drawing, hull, idx, color, 1, 8, vector<Vec4i>(), 0, Point());
        }
        

        imshow("contours", drawing);


        Mat drawing_G;
        cvtColor(drawing, drawing_G, COLOR_BGR2GRAY);

        // 방법 2
	    vector<Vec4i> lines;    
    	HoughLinesP(drawing_G, lines, 1, CV_PI / 180, 5, 20, 0);

        cvtColor(drawing_G, drawing_G, COLOR_GRAY2BGR);

    	
    	Mat dst(Size(640, 480), CV_8UC3);

        vector<float> theta_list; 
        theta_list.reserve(30);


	    for (const Vec4i& l : lines)
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

            std_msgs::Float64 ParkingAngle_RIGHT_msg;

            if (theta > -90 && theta < -70)
            {
                theta = -theta;
                line(drawing_G, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 40, LINE_AA);
                circle(drawing_G, Point(l[0], l[1]), 3, Scalar(255, 0, 0), -1);
                // if(theta < min_theta) min_theta = theta;
                theta_list.push_back(theta);
                // cout << "theta_original: " << theta << "\n" << endl;
            }  
            
            else if (theta > 30 && theta < 100)
            {
                line(drawing_G, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 40, LINE_AA);
                circle(drawing_G, Point(l[0], l[1]), 3, Scalar(255, 0, 0), -1);
                // if(theta < min_theta) min_theta = theta;
                theta_list.push_back(theta);
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

        float theta_sum = 0;
        for(const auto& n : theta_list) {
            theta_sum += n;
        }

    // theta 평균 구하기
    theta_sum /= theta_list.size();

    final_theta = theta_sum;

    if (final_theta > 52 && final_theta < 88)
    {
        ParkingAngle_RIGHT_msg.data = final_theta;
		parking_level3_pub.publish(ParkingAngle_RIGHT_msg);

    }

    else if(final_theta < 60)
    {
        right_level = 1;
    }


    imshow("line_detect", line_detect);
    imshow("dst", drawing_G);

    Mat edge_resize;
    resize(drawing_G, edge_resize, Size(cols, rows), 0, 0, INTER_CUBIC);
    imshow("edge_resize", edge_resize);

    // Mat edge_final = Mat::zeros(480, 640, CV_8UC3);
        
    return edge_resize;

}

Mat front_level(Mat img)    // 기존의 preprocessing_forth_level, 전방 라인 검출
{
    // contours
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(src_HLS, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
    Scalar color(rand() & 255, rand() & 255, rand() & 255);


    vector< vector<Point> > hull(contours.size());

    for(int i = 0; i < contours.size(); i++)
        convexHull(Mat(contours[i]), hull[i], false);

    Mat drawing = Mat::zeros( edge.size(), CV_8UC3 );
    // 모든 외각선 그리기
    for(int idx = 0; idx >= 0; idx = hierarchy[idx][0])
    {
        // drawContours( drawing, contours, idx, color, 2, LINE_8, hierarchy);
        drawContours(drawing, hull, idx, color, 1, 8, vector<Vec4i>(), 0, Point());
    }
    

    imshow("contours", drawing);


    Mat drawing_G;
    cvtColor(drawing, drawing_G, COLOR_BGR2GRAY);

    // 방법 2
    vector<Vec4i> lines;    
    HoughLinesP(drawing_G, lines, 1, CV_PI / 180, 5, 20, 0);

    cvtColor(drawing_G, drawing_G, COLOR_GRAY2BGR);

    
    Mat dst(Size(640, 480), CV_8UC3);

    vector<float> theta_list; 
    theta_list.reserve(30);


    for (const Vec4i& l : lines)
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

        std_msgs::Float64 ParkingAngle_RIGHT_msg;

        if (theta > -90 && theta < -70)
        {
            theta = -theta;
            line(drawing_G, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 40, LINE_AA);
            circle(drawing_G, Point(l[0], l[1]), 3, Scalar(255, 0, 0), -1);
            // if(theta < min_theta) min_theta = theta;
            theta_list.push_back(theta);
            // cout << "theta_original: " << theta << "\n" << endl;
        }  
        
        else if (theta > 30 && theta < 100)
        {
            line(drawing_G, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 40, LINE_AA);
            circle(drawing_G, Point(l[0], l[1]), 3, Scalar(255, 0, 0), -1);
            // if(theta < min_theta) min_theta = theta;
            theta_list.push_back(theta);
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

    float theta_sum = 0;
    for(const auto& n : theta_list) {
        theta_sum += n;
    }

// theta 평균 구하기
theta_sum /= theta_list.size();

final_theta = theta_sum;

if (final_theta > 52 && final_theta < 88)
{
    ParkingAngle_RIGHT_msg.data = final_theta;
    parking_level3_pub.publish(ParkingAngle_RIGHT_msg);

}

else if(final_theta < 60)
{
    right_level = 1;
}


imshow("line_detect", line_detect);
imshow("dst", drawing_G);

Mat edge_resize;
resize(drawing_G, edge_resize, Size(cols, rows), 0, 0, INTER_CUBIC);
imshow("edge_resize", edge_resize);

// Mat edge_final = Mat::zeros(480, 640, CV_8UC3);
    
return edge_resize;

}
