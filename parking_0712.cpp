#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <algorithm>
#include <iostream>
#include <stdio.h>
#include <math.h> 
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "std_msgs/Int32.h" // Int32형 메시지 주고받을 수 있게 함
#include <std_msgs/Float64.h> //Float64형 메시지 주고방르 수 있게 함
#include <sstream> //C++ 표준 라이브러리 일종
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h> // ros에서 opencv사용할 수 있게 함

#include <vector>
#include <cmath>

/*################################################################################################################*/
//parking_level1 = 컨투어 사이즈 측정으로 주차해야 하는 공간 파악
//	컨투어 사이즈 값 변경 line=136
//	화면 HLS 변경 line=120
//parking_level2 = 오른쪽 주차선 파악
//parking_level3 = 정면 주차선 각도 파악
/*################################################################################################################*/

// 어떤 자료형이든 문자형으로 바꿈
#define SSTR( x ) static_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x) ).str()

#define PI 3.141592

using namespace cv;
using namespace cv::ml;
using namespace std;

void on_mouse1(int event, int x, int y, int flags, void*); //좌표출력용 콜백함수
void on_mouse2(int event, int x, int y, int flags, void*); //색 추출용 콜백함수
void on_HLS_change(int, void*);
void imageCallback(const sensor_msgs::ImageConstPtr &msg);
Mat bird_eyes_view_for_front_camara(Mat img);
Mat bird_eyes_view_inverse_for_front_camara(Mat img);
Mat mask_filter(Mat img, int _mask_w, int _mask_h, int thresh);

Mat bird_eyes_view(Mat img);

Mat level_one_src;	//first level에서 src영상
Mat frame_HLS; 		//first level에서 HLS 색변환 영상 복제품 (line=112)
Mat mask; 			//first level에서 inRange 한 후의 HLS영상 (line=119)
Mat img(Size(640, 480), CV_8UC3, Scalar(0,0,0));

int lower_H = 0, upper_H = 0;
int lower_L = 0, upper_L = 0;
int lower_S = 0, upper_S = 0;

Point ptOld1;

float min_theta = 90;
float final_theta = 90;

int isStop = 10;
int right_line = 0;     // 기존의 SecondLevel
bool parking1 = false;
bool parking2 = false;
float encoder = 3;
bool enc_fail = false;
bool callback = false;
bool third_level = false;
bool FinalLevel;
bool third_stop = false;
Point rec_center;
Mat img_level3;
Mat img_level3_new;
Mat frame_for_front_camara;
Mat img_resize1000;
Mat warp_matrix_inv;

// ros::Publisher parking_pub;
ros::Subscriber Parking_level1_sub;		// 라이다
ros::Publisher parking_level2_pub;		// 1번선 각도
ros::Publisher parking_level3_angle_pub;		// 2번선 각도
ros::Publisher parking_level3_sign_pub;		// 오른쪽 or 왼쪽 틀기
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

Point parking_level1(Mat frame, int delay)
{
    

    level_one_src = frame.clone();
    // 이미지 자르기
    rectangle(level_one_src, Rect(0, 0, 640, 70), Scalar(0, 0, 0), -1); //상단 for koreatech
	rectangle(level_one_src, Rect(0, 0, 640, 110), Scalar(0, 0, 0), -1); //for k-citys
	// rectangle(level_one_src, Rect(0, 380, 640, 480), Scalar(0, 0, 0), -1); //하단


    // imshow("src", level_one_src);
    
    // HLS색 공간 변경
    Mat src_HLS;
    cv::cvtColor(frame1, src_HLS, COLOR_BGR2HLS);
    
    frame_HLS = src_HLS.clone();

    imshow("HLS", frame_HLS);
    setMouseCallback("HLS", on_mouse2);
    Mat binary;

    inRange(frame_HLS, Scalar(0, 120, 0), Scalar(255, 255, 255), mask);
    // imshow("mask", mask);

    Mat mask2 = mask.clone();

    // contours (외각선 추출)
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask2, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
    Scalar color(rand() & 255, rand() & 255, rand() & 255); //색상은 아름답게
    int lrgctridx = 0;
    int minarea = 48000;
    int nowarea = 0;
    for (int i = 0; i < contours.size(); i++)
    {
        double a = contourArea(contours[i]);
        // 모든 외각선의 사이즈 출력
        // cout << "contour idx = " << i << " " << "size = " << a << endl;
        if (a > 10000 & a < 47000) //컨투어 영역 조건문
        {
            nowarea = a;
            if(a < minarea) //가장 작은 값을 저장
            {
                minarea = a;
                lrgctridx = i;
				cout << "lrgctridx = " << i << " " << "minarea = " << minarea << endl;
            }
            
        }
    }

	// 그림그릴 공간 생성
    Mat drawing = Mat::zeros( mask2.size(), CV_8UC3 );

	// 외각 껍대기 그리기
    vector<Point> hull;
    convexHull(Mat(contours[lrgctridx]), hull, false);

	// 영상에 그리기위한 과정
    vector<vector<Point>> fake_hull;
    fake_hull.push_back(hull);

    drawContours( drawing, contours, lrgctridx, color, 2, LINE_8, hierarchy);
    drawContours(drawing, fake_hull, 0, color, 2, LINE_8);
    
	//#######################################################################################################################//
	// 최상단점, 최하단점 찾기
	//######################################################################################################################//

    int top_x = hull[0].x;
	int top_y = hull[0].y;
    int top_num = 0;

	int bottom_x = hull[0].x;
    int bottom_y = hull[0].y;
    int bottom_num = 0;


    for(int i = 0; i < hull.size(); i++)
    {
        if(sqrt(pow(hull[i].x - 640, 2) + pow(hull[i].y - 0, 2)) <= sqrt(pow(top_x - 640, 2) + pow(top_y - 0, 2)))
        {
            top_x = hull[i].x;
			top_y = hull[i].y;
            top_num = i;
        }
        if(sqrt(pow(hull[i].x - 640, 2) + pow(hull[i].y - 480, 2)) <= sqrt(pow(top_x - 640, 2) + pow(top_y - 480, 2)))
        {
			bottom_x = hull[i].x;
            bottom_y = hull[i].y;
            bottom_num = i;
        }
    }

    int mean_x = int((hull[top_num].x + hull[bottom_num].x) / 2);
    int mean_y = int((hull[top_num].y + hull[bottom_num].y) / 2);

    circle(drawing, hull[top_num], 10, Scalar(255, 0, 0), -1);
    circle(drawing, hull[bottom_num], 10, Scalar(0, 0, 255), -1);
    circle(drawing, Point(mean_x, mean_y), 10, Scalar(0, 255, 0), -1);

	// 최종 출력 영상
	addWeighted(level_one_src, 1, drawing, 1, 0, drawing);
    imshow("first_level", drawing);

    Point final(mean_x, mean_y);
    return final;

	//level_one 끝
    waitKey(1);
}

Mat parking_level3(Mat img)
{

	img_level3 = img.clone();

	Mat HSV_image;
	cvtColor(img, HSV_image, COLOR_BGR2HSV);

	Scalar lower_white = Scalar(0, 0, 170);
	Scalar upper_white = Scalar(255, 100, 255);

	Mat mask_level3;
	inRange(HSV_image, lower_white, upper_white, mask_level3);

	Mat img_warp1;
	img_warp1 = bird_eyes_view_for_front_camara(mask_level3);

	Mat img_warp2 = img_warp1.clone();

	// 소벨마스크 엣지검출
	Mat dx, dy;
	Sobel(img_warp1, dx, CV_32FC1, 1, 0);
	Sobel(img_warp1, dy, CV_32FC1, 0, 1);

	Mat fmag, mag;
	magnitude(dx, dy, fmag);
	fmag.convertTo(mag, CV_8UC1);
	Mat edge = mag > 130;

	//#######################################################################################################
	// 허프라인 통한 전방각도 추출 시작
	vector<Vec4i> lines;
	HoughLinesP(edge, lines, 1, CV_PI / 180, 160, 50, 5);

	Mat dst;
	cvtColor(edge, dst, COLOR_GRAY2BGR);
	Mat dst4(Size(640, 480), CV_8UC1);

	for (Vec4i l : lines)
	{
		// line(dst,Point(l[0],l[1]),Point(l[2],l[3]),Scalar(0,0,255),2,LINE_AA);
		//cout << "lines.size: " << lines.size() << endl;
		float dx = l[2] - l[0], dy = l[3] - l[1];
		float k = dy / dx;
		float radian = atan(k);
		float theta = (radian * 180) / PI;
		float d = sqrt(pow(dx, 2) + pow(dy, 2));
		float y1 = l[1] - 10;
		float y2 = l[3] + 10;
		float x0 = (l[0] + l[2]) / 2;
		float y0 = (l[1] + l[3]) / 2;
		// cout << "theta: " << theta << endl;
		std_msgs::Float64 Plus_or_Minus_msg;

		if ((theta > -20 && theta < 20) && (d > 70))
		{
			line(dst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2, LINE_AA);
			// cout << "theta: " << theta << endl;
			//cout << "Point(l[0], l[1]): " << Point(l[0], l[1]) << " Point(l[2], l[3]): " << Point(l[2], l[3]) << endl;
			line(dst4, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), 15, LINE_AA);
			// cout << "theta_original: " << theta << endl;
			Plus_or_Minus_msg.data = theta;
			parking_level3_sign_pub.publish(Plus_or_Minus_msg);
			cout << "Plus or Minus" << endl;
			ROS_INFO("%f", Plus_or_Minus_msg.data);
		}
	}
	// imshow("dst", dst);
	// imshow("dst4", dst4);

	int num = (int)(dst4.total() * 0.1);
	for (int i = 0; i < num; i++)
	{
		int x44 = rand() % dst4.cols;
		int y44 = rand() % dst4.rows;
		dst4.at<uchar>(y44, x44) = (i % 2) * 255;
	}

	Mat dst_final_blur4;
	medianBlur(dst4, dst_final_blur4, 29);
	// imshow("dst_final_blur4",dst_final_blur4);

	Mat labels4, stats4, centroids4;
	int cnt4 = connectedComponentsWithStats(dst_final_blur4, labels4, stats4, centroids4);

	Mat dst_4;
	cvtColor(dst_final_blur4, dst_4, COLOR_GRAY2BGR);

	for (int i = 1; i < cnt4; i++)
	{
		int *p = stats4.ptr<int>(i);
		// rectangle(dst_4, Rect(p[0], p[1], p[2], p[3]), Scalar(0, 0, 255), 2);
		// cout << "stats1 : " << p[4] << endl;
		// float theta = (atan((float)p[3] / p[2]) * 180) / PI;
		float x1 = p[0], y1 = p[1], x2 = p[0] + p[2], y2 = p[1] + p[3];
		float k1 = ((y2 - y1) / (x2 - x1));
		float radian1 = atan(k1);
		float theta_live = (radian1 * 180) / PI;

		// cout<<"theta: "<<theta<<endl;
		std_msgs::Float64 ParkingAngle_FRONT_msg;
		if (p[4] > 5000 && p[4] < 10000) // 8000~20000

		{
			rectangle(dst_4, Rect(p[0], p[1], p[2], p[3]), Scalar(0, 0, 255), 2);
			// cout << "stats1 : " << p[4] << endl;
			//cout << "p[2]: " << p[2] << " p[3]: " << p[3] << endl;
			// cout << "theta_live: " << theta << endl;
			ParkingAngle_FRONT_msg.data = theta_live;
			parking_level3_angle_pub.publish(ParkingAngle_FRONT_msg);
			cout << "frontangle" << endl;
			ROS_INFO("%f", ParkingAngle_FRONT_msg.data);
			if (theta_live < 9)
			{
				cout << "stop change angle!" << endl;
			}
		}
	}
	// imshow("dst_4", dst_4);


	Mat warp_inv1;
	warp_inv1 = bird_eyes_view_inverse_for_front_camara(dst_4);
	// imshow("warp_inv", warp_inv);

	// 허프라인 검출통한 기울기 구하기 끝
	//################################################################################################

	Mat img_integral;
	cv::integral(img_warp2, img_integral);

	// 전방거리 계산
	Mat img_mask;
	img_mask = mask_filter(img_integral, 5, 5, 95);
	//imshow("img_mask", img_mask);

	Mat warp_inv2;
	warp_inv2 = bird_eyes_view_inverse_for_front_camara(img_mask);
	// imshow("warp_inv11", warp_inv11);

	Mat final2;
	cv::resize(warp_inv2, final2, cv::Size(640, 480), 0, 0);
	// imshow("img_resize",final2);

	Mat final3;
	cv::resize(warp_inv1, final3, cv::Size(640, 480), 0, 0);


	Mat final4 = final2 + final3;


	Mat final_level3;
	addWeighted(img_level3, 0.5, final4, 0.5, 0, final_level3);

	int count = 100;

	if (isStop == 1)
	{
		count = 1;
		FinalLevel = 1;
		//putText(final10, "STOP!!", Point(380, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
	}
	else
	{
		count = isStop;
	}

	std_msgs::Float64 stopline_msg;
	stopline_msg.data = count;
	cout << "stopline" << endl;
	ROS_INFO("%f", stopline_msg.data);
	cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1" << endl; // data 메시지를표시한다
	stopline_pub.publish(stopline_msg);									// 메시지를발행한다

	// imshow("final10", final10);

	return final_level3;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "parking_publisher");
	ros::NodeHandle nh;

    Parking_level1_sub = nh.subscribe("Lidar_Stop", 10, parking_cb); //parkingstop
	Parking_encoder_fail_sub = nh.subscribe("enc_fail", 10, enc_fail_cb);
	parking_level2_pub = nh.advertise<std_msgs::Float64>("ParkingAngle_RIGHT", 10);
	parking_level3_angle_pub = nh.advertise<std_msgs::Float64>("ParkingAngle_FRONT", 10);
	parking_level3_sign_pub = nh.advertise<std_msgs::Float64>("Plus_or_Minus", 10);
	stopline_pub = nh.advertise<std_msgs::Float64>("stopline_parking", 100);
	encoder_pub = nh.advertise<std_msgs::Float64>("encoder_mode", 100);
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub_image = it.subscribe("/camera/vision/image_raw", 100, imageCallback);
	image_transport::Publisher image_raw_pub = it.advertise("camera/vision2/image_raw", 100);
	sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(50);
    printf("Waiting for ---/camera/stopline/image_raw---\n");

	VideoCapture cap(4);

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    string trackerTypes[7] = {"BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW", "MOSSE", "CSRT"};

    string trackerType = trackerTypes[0]; // BOOSTING 모드 선정

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

    Mat frame, frame3;

    while(ros::ok())
    {
		if(callback == true)
		{
			if (!cap.isOpened())
			{
				cerr << "finish!\n" << endl;
			}

			cap >> frame;

			Mat frame_re;
			resize(frame, frame_re, Size(640, 480), 0, 0, INTER_CUBIC);

			msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
			
			image_raw_pub.publish(msg);

			Mat img_resize6;
			cv::resize(frame_for_front_camara, img_resize6, cv::Size(640, 480), 0, 0);
			imshow("front_camera",frame_for_front_camara);
			img_resize1000 = img_resize6.clone();

			std_msgs::Float64 encoder_mode_msg;

			double fps = cap.get(CAP_PROP_FPS);
			cout << "FPS: " << fps << endl;
			int delay = cvRound(1000 / fps);

			int count = 10;

			// parking_level1
			if (parking1 == false)
			{
				rec_center = parking_level1(frame_re, delay);
			}  

			// parking_level2
			else if(parking1 == true && third_level == false)
			{
				
				// erp 엔코더 실행
				encoder = 2;

				//트래킹 위치 선정
				Rect2d bbox(Point(rec_center.x-10, rec_center.y - 10), Point(rec_center.x + 30, rec_center.y + 10));
				// rectangle(frame_re, bbox, Scalar( 255, 0, 0 ), 2, 1 );


				// imshow("Tracking", frame_re);
				tracker->init(frame_re, bbox);

				double timer = (double)getTickCount();
				bool ok = tracker->update(frame_re, bbox);

				// fps 계산
				float fps = getTickFrequency() / ((double)getTickCount() - timer);

				// 마스크용 영상
				Mat mask = Mat::zeros(480, 640, CV_8UC1);

				// line detect video
				Mat line_detect = Mat::zeros(480, 640, CV_8UC3);

				Mat final = frame_re.clone();

				Mat frame_part;

				if (ok)
				{
					rectangle(final, bbox, Scalar( 255, 0, 0 ), 2, 1);
					rectangle(mask, bbox, Scalar( 255, 255, 255), -1);
					frame_part = frame_re(bbox);
					// frame.copyTo(line_detect, mask);

				}
				else
				{
					putText(final, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);
				}

				putText(final, trackerType + " Tracker", Point(100,20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50),2);

				putText(final, "FPS : " + SSTR(int(fps)), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);

				// imshow("Tracking", frame_re);
				// imshow("frame_part", frame_part);

				int cols = frame_part.cols;
				int rows = frame_part.rows;

				// 큰 사이즈로 확대
				Mat big_frame_part;
				resize(frame_part, big_frame_part, Size(cols*15, rows*15), 0, 0, INTER_CUBIC);
				// imshow("big_frame_part", big_frame_part);

				// imshow("MASK", mask);

				// HLS 색공간으로 이진화
				Mat src_HLS;
				cvtColor(big_frame_part, src_HLS, COLOR_BGR2HLS);
				inRange(src_HLS, Scalar(0, 130, 0), Scalar(255, 255, 255), src_HLS);

				// 이진화 모폴로지 연산
				dilate(src_HLS, src_HLS, Mat());
				erode(src_HLS, src_HLS, Mat());
				dilate(src_HLS, src_HLS, Mat());

				// 흑백영상으로 바꿈
				Mat line_detect_G;
				cvtColor(line_detect, line_detect_G, COLOR_BGR2GRAY);

				// imshow("HLS", src_HLS);

				// 소벨 마스크
				Mat dx, dy;
				Sobel(src_HLS, dx, CV_32FC1, 1, 0);
				Sobel(src_HLS, dy, CV_32FC1, 0, 1);

				Mat fmag, mag;
				magnitude(dx, dy, fmag);
				fmag.convertTo(mag, CV_8UC1);

				Mat edge = mag > 150;

				// imshow("edge", edge);

				// contours
				vector<vector<Point>> contours;
				vector<Vec4i> hierarchy;
				findContours(src_HLS, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
				Scalar color(rand() & 255, rand() & 255, rand() & 255);
				int lrgctridx = 0;
				int maxarea = 0;
				int nowarea = 0;
				for (int i = 0; i < contours.size(); i++)
				{
					double a = contourArea(contours[i]);
					// 모든 외각선의 사이즈 출력
					// cout << "contour idx = " << i << " " << "size = " << a << endl;
					nowarea = a;
					if(a > maxarea)
					{
						maxarea = a;
						lrgctridx = i;
					}
				}

				vector<Point> hull;
			

				// ########################################## error occur!!!!!!!!!!!!!!!! #################################################################

				// Mat drawing; 
				// drawing = Mat::zeros( edge.size(), CV_8UC3 );

				// drawContours( drawing, contours, lrgctridx, Scalar(255, 255, 255), 2, LINE_8, hierarchy);

				// imshow("drawing", drawing);

				// waitKey();

				//############################################################################################################################################
				//tracker 영역이 하늘이나 배경 등 엣지를 감지할 수 없는 공간을 잡아버리면 엣지선이 검출되지 않기 때문에 컨투어 함수를 써도 외각선이 검출되지 않아 convexhull 함수에서 오류가 발생
				//############################################################################################################################################

				cout << contours[lrgctridx] << endl;

				convexHull(Mat(contours[lrgctridx]), hull, false);


				vector<vector<Point>> fake_hull;

				fake_hull.push_back(hull);

				Mat drawing; 
				drawing = Mat::zeros( edge.size(), CV_8UC3 );

				// drawContours( drawing, contours, idx, color, 2, LINE_8, hierarchy);
				drawContours(drawing, fake_hull, 0, Scalar(255, 255, 255), 2, LINE_8);

				cvtColor(drawing, drawing, CV_BGR2GRAY);

				// 주차선 기울기 구하기

				Point top_P(0,0);
				Point top_temp_P;
				Point low_P(0,449);
				Point low_temp_P;

				cout << hull << endl;

				Point a;
				Point b;
				bool aaa = true;
				bool bbb = true;

				for(int j=5; j < drawing.rows ; j++)
				{	
					for(int i=3; i < drawing.cols ;i++)
					{
						if(drawing.at<uchar>(j, i) == 255)
						{	
							if(i != drawing.cols)
							{
								if(i < 3)
								{
									break;
								}
								else
								{
									b.x = i;
									b.y = j;
									bbb = false;
									break;
								}
							}
						}
					}
					if(bbb == false)
					{
						break;
					}
				}

				for(int j=drawing.rows-5; j > 0; j--)
				{	
					for(int i=3; i < drawing.cols ;i++)
					{
						if(drawing.at<uchar>(j, i) == 255)
						{	
							if(i != drawing.cols)
							{ 
								if(i < 3)
								{
									break;
								}
								else
								{
									a.x = i;
									a.y = j;
									aaa = false;
									break;
								}
							}
						}
					}
					if(aaa == false)
					{
						break;
					}
				}
				
				circle(drawing, a, 30, 200, -1);
				circle(drawing, b, 30, 200, -1);

				cvtColor(drawing, drawing, CV_GRAY2BGR);


				// cout << "top_P" << top_P;
				// cout << "low_P" << low_P;

				line(drawing, top_P, low_P, Scalar(0, 0, 255), 3);
				imshow("drawing", drawing);

				float ddx = float(b.x) - float(a.x), ddy = float(b.y) - float(a.y);
				float k = ddy / ddx;
				float radian = atan(k);
				float theta = (radian * 180) / CV_PI;

				float theta_temp;

				if (theta > -90 && theta < -30)
				{
					// 둔각으로 출력
					theta_temp = (180 + theta);
				}
				else if (theta > 30 && theta < 90)
				{
					theta_temp = theta;
				}
				else
				{
					theta_temp = 90;
				}

				Mat drawing_G = drawing.clone();
				
				Mat dst(Size(640, 480), CV_8UC3);

				final_theta = theta_temp;

				std_msgs::Float64 parking_level2_msg;

				parking_level2_msg.data = theta_temp;
				parking_level2_pub.publish(parking_level2_msg);

				//#################################################################################################//
				// 각도조건
				//#################################################################################################//

				if(final_theta <= 45) 
				{
					third_level = true;
					// parking2 = false;
				}

				// imshow("line_detect", line_detect);
				// imshow("dst", drawing_G);
				// imshow("drawing", drawing);

				//원래 크기로 축소
				Mat edge_resize;
				resize(drawing_G, edge_resize, Size(cols, rows), 0, 0, INTER_CUBIC);
				// imshow("edge_resize", edge_resize);

				Mat edge_final = Mat::zeros(480, 640, CV_8UC3);
				
				// 최종 영상에 허프라인 결과 합성
				Mat final_roi = final(bbox);
				addWeighted(final_roi, 0.3, edge_resize, 1, 0, final_roi);

				
				// final = final + edge;

				putText(final, "theta_live : " + SSTR(float(final_theta)), Point(300,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);

				imshow("final", final);

				waitKey(1);
			}

			// parking_level3
			else if (third_level == true && third_stop == false)
			{

				// erp 엔코더 계속 실행
				encoder = 2;
				
				frame3 = parking_level3(img_resize6);
				// imshow("frame3",frame3);
				Mat final2;
				addWeighted(img_resize1000, 0.5, frame3, 0.5, 0, final2);

				// 거리 화면에 출력
				putText(final2, SSTR(float(isStop)) + "M", Point(300,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 5);
				count = isStop;

				if (isStop == 10)
				{
					count = 10;
					putText(final7, "GO!", Point(380, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
				}

				imshow("final2", final2);

				if (FinalLevel == 1 || enc_fail == true)
				{
					third_stop = true;
					cout << "enc_fail : " << enc_fail << endl;
					cout << "FinalLevel : " << FinalLevel << endl;
					encoder = 1;
				}
			}

			encoder_mode_msg.data = encoder;
			encoder_pub.publish(encoder_mode_msg);

			if (waitKey(1) == 27)
			{
				break;
				printf("end");
			}

		}
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
		cerr << "[H, L, S] : " << frame2.at<Vec3b>(y, x) << "\n" << endl;
		// cerr << "v : " << hsv[2] << "\n" << endl;
		break;
	}
}

void on_HLS_change(int, void*)
{
   Scalar lowerb(lower_H, lower_L, lower_S);
   Scalar upperb(upper_H, upper_L, upper_S);
   inRange(frame2, lowerb, upperb, mask);

   imshow("mask", mask);
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	printf("이미지 수신중\n");
	Mat image1 = cv_bridge::toCvShare(msg, "bgr8")->image;
	frame_for_front_camara = image1.clone();
	cv::waitKey(1);
	callback = true;
	imshow("image1", image1);
}

Mat bird_eyes_view_for_front_camara(Mat img)
{
	int width = img.cols;
	int height = img.rows;

	width = img.cols;
	height = img.rows;
	Mat warp_matrix;
	Point2f warp_src_point[4];
	Point2f warp_dst_point[4];

	// warp_src_point[0].x = 26;
	// warp_src_point[0].y = 395;
	// warp_src_point[1].x = 635;
	// warp_src_point[1].y = 420;
	// warp_src_point[2].x = 26;
	// warp_src_point[2].y = 120;
	// warp_src_point[3].x = 635;
	// warp_src_point[3].y = 120;

	warp_src_point[0].x = 15;
	warp_src_point[0].y = 320;
	warp_src_point[1].x = 630;
	warp_src_point[1].y = 320;
	warp_src_point[2].x = 220;
	warp_src_point[2].y = 140;
	warp_src_point[3].x = 470;
	warp_src_point[3].y = 140;

	// warp_dst_point[0].x = 150;
	// warp_dst_point[0].y = height * 0.8;bird_eyes_view_inverse
	// warp_dst_point[2].x = 150;
	// warp_dst_point[2].y = 0;
	// warp_dst_point[3].x = width - 150;
	// warp_dst_point[3].y = 0;

	warp_dst_point[0].x = 150;
	warp_dst_point[0].y = height;
	warp_dst_point[1].x = width - 150;
	warp_dst_point[1].y = height;
	warp_dst_point[2].x = 150;
	warp_dst_point[2].y = 0;
	warp_dst_point[3].x = width - 150;
	warp_dst_point[3].y = 0;

	warp_matrix = cv::getPerspectiveTransform(warp_src_point, warp_dst_point);

	Mat dst;
	cv::warpPerspective(img, dst, warp_matrix, cv::Size(width, height));
	//imshow("dst", dst); top view

	return dst;
}

Mat bird_eyes_view_inverse_for_front_camara(Mat img)
{
	int width = img.cols;
	int height = img.rows;

	width = img.cols;
	height = img.rows;
	Mat warp_matrix;
	Point2f warp_src_point[4];
	Point2f warp_dst_point[4];

	// warp_src_point[0].x = 26;
	// warp_src_point[0].y = 395;
	// warp_src_point[1].x = 635;
	// warp_src_point[1].y = 420;
	// warp_src_point[2].x = 26;
	// warp_src_point[2].y = 120;
	// warp_src_point[3].x = 635;
	// warp_src_point[3].y = 120;

	warp_src_point[0].x = 15;
	warp_src_point[0].y = 320;
	warp_src_point[1].x = 630;
	warp_src_point[1].y = 320;
	warp_src_point[2].x = 220;
	warp_src_point[2].y = 140;
	warp_src_point[3].x = 470;
	warp_src_point[3].y = 140;

	// warp_dst_point[0].x = 150;
	// warp_dst_point[0].y = height * 0.8;
	// warp_dst_point[1].x = width - 150;
	// warp_dst_point[1].y = height * 0.8;
	// warp_dst_point[2].x = 150;
	// warp_dst_point[2].y = 0;
	// warp_dst_point[3].x = width - 150;
	// warp_dst_point[3].y = 0;

	warp_dst_point[0].x = 150;
	warp_dst_point[0].y = height;
	warp_dst_point[1].x = width - 150;
	warp_dst_point[1].y = height;
	warp_dst_point[2].x = 150;
	warp_dst_point[2].y = 0;
	warp_dst_point[3].x = width - 150;
	warp_dst_point[3].y = 0;

	warp_matrix = cv::getPerspectiveTransform(warp_src_point, warp_dst_point);
	invert(warp_matrix, warp_matrix_inv);

	Mat dst1;
	cv::warpPerspective(img, dst1, warp_matrix_inv, cv::Size()); //������̺� ����ȯ
	//imshow("dst1", dst1);

	return dst1;
}

Mat mask_filter(Mat img, int _mask_w, int _mask_h, int thresh)
{
	int height = img.rows;
	int width = img.cols;
	Mat img_maskfilter;
	img_maskfilter = Mat::zeros(height, width, CV_8UC1);
	Mat img_stop;
	img_stop = Mat::zeros(height, width, CV_8UC3);
	float mask[3];
	int sx = 0;
	isStop = 10;

	putText(img_stop, "7M", Point(0, 40), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
	line(img_stop, Point(3, 40), Point(600, 40), Scalar(0, 0, 255));
	putText(img_stop, "6M", Point(0, 140), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255));
	line(img_stop, Point(3, 140), Point(600, 140), Scalar(255, 0, 255));
	putText(img_stop, "5M", Point(0, 240), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255));
	line(img_stop, Point(3, 240), Point(600, 240), Scalar(0, 255, 255));
	putText(img_stop, "4M", Point(0, 340), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 0));
	line(img_stop, Point(3, 340), Point(600, 340), Scalar(255, 255, 0));
	putText(img_stop, "3M", Point(0, 440), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
	line(img_stop, Point(3, 440), Point(600, 440), Scalar(0, 0, 255));

	uint *image = (uint *)img.data;
	uchar *score_data = (uchar *)img_maskfilter.data;
	int mask_w = _mask_w, mask_h = _mask_h;

	int sy = 0;

	int roi_w = 100;
	int histo = 0;
	for (int y = 20; y < height - 17; y++)
	{
		histo = 0;
		for (int x = int(width / 2) - roi_w; x <= int(width / 2) + roi_w; x++)
		{
			for (int i = 0; i < 3; i++)
			{
				sy = y + (2 * mask_h + 1) * (i - 1);
				int dx, cx, bx, ax;
				int dy, cy, by, ay;
				dy = sy + mask_h;
				dx = x + mask_w;
				cy = sy - mask_h - 1;
				cx = x + mask_w;
				by = sy + mask_h;
				bx = x - mask_w - 1;
				ay = sy - mask_h - 1;
				ax = x - mask_w - 1;
				mask[i] = image[(dy)*width + dx] - image[(cy)*width + cx] - image[(by)*width + bx] + image[(ay)*width + ax];
			}

			float sum = ((mask[1] - mask[0]) + (mask[1] - mask[2])) / 2;
			if (sum > 10000)
			{
				score_data[width * y + x] = 255;
				histo++;
			}
		}

		if (histo > thresh)
		{
			line(img_stop, Point(int(width / 2) - roi_w, y), Point(int(width / 2) + roi_w, y), Scalar(255, 0, 0), 30);
			if (y < 140)
			{
				cout << "stop line distance : 5M\n"
					 << endl;
				isStop = 5;
			}
			else if (y < 230)
			{
				cout << "stop line distance : 4M\n"
					 << endl;
				isStop = 4;
			}
			else if (y < 267)
			{
				cout << "stop line distance : 3M\n"
					 << endl;
				isStop = 3;
			}
			else if (y < 330)
			{
				cout << "stop line distance : 2M\n"
					 << endl;
				isStop = 2;
			}
			else if (y > 400)
			{
				cout << "stop line!!!!\n"
					 << endl;
				isStop = 1;
			}
			break;
		}
	}
	// imshow("img_stop",img_stop);
	return img_stop;
}