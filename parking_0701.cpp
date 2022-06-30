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

/*################################################################################################################*/
//parking_level1 = 컨투어 사이즈 측정으로 주차해야 하는 공간 파악
//parking_level2 = 오른쪽 주차선 파악
//parking_level3 = 정면 주차선 각도 파악
/*################################################################################################################*/

// 어떤 자료형이든 문자형으로 바꿈
#define SSTR( x ) static_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x) ).str()

#define PI 3.141592

using namespace cv;
using namespace cv::ml;
using namespace std;

// void parking_cb(const std_msgs::Bool::ConstPtr &msgs);
// void imageCallback(const sensor_msgs::ImageConstPtr &msg);
void on_mouse1(int event, int x, int y, int flags, void*);
void on_mouse2(int event, int x, int y, int flags, void*);
void on_HLS_change(int, void*);
void imageCallback(const sensor_msgs::ImageConstPtr &msg);
Mat bird_eyes_view_for_front_camara(Mat img);
Mat bird_eyes_view_inverse_for_front_camara(Mat img);
Mat mask_filter(Mat img, int _mask_w, int _mask_h, int thresh);

Mat bird_eyes_view(Mat img);

Mat frame1, frame2, mask;
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
float encoder = 3;
bool enc_fail = false;
bool callback = false;
bool third_level = false;
bool FinalLevel;
Point rec_center;
Mat img_color;
Mat img_resize_new;
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
    
    frame1 = frame.clone();
    // 이미지 자르기
    // rectangle(frame1, Rect(0, 0, 640, 190), Scalar(0, 0, 0), -1);


    imshow("src", frame1);
    
    // 히스토그램 평활화
    // Mat src_ycrcb;
    // cv::cvtColor(src, src_ycrcb, COLOR_BGR2YCrCb);
    // vector<Mat> ycrcb_planes;
    // split(src_ycrcb, ycrcb_planes);
    // equalizeHist(ycrcb_planes[0], ycrcb_planes[0]);
    // Mat dst_ycrcb;
    // merge(ycrcb_planes, dst_ycrcb);
    // Mat dst;
    // cvtColor(dst_ycrcb, dst, COLOR_YCrCb2BGR);
    
    // YUV색 공간 변경
    Mat src_HLS;
    cv::cvtColor(frame1, src_HLS, COLOR_BGR2HLS);
    // 가우시안 블러
    // Mat gaussian;
    // GaussianBlur(src_YUV, gaussian, Size(), 1);
    
    frame2 = src_HLS.clone();
    // frame2 = src_YUV.clone();
    imshow("HLS", frame2);
    setMouseCallback("HLS", on_mouse2);
    Mat binary;
    // inRange(frame2, Scalar(190, 122, 128), Scalar(255, 255, 255), mask);
    
    // 모폴로지 연산
    // dilate(mask, mask, Mat(), Point(-1, -1), 1);
    // erode(mask, mask, Mat(), Point(-1, -1), 1);
    // dilate(mask, mask, Mat(), Point(-1, -1), 1);
    
    // erode(mask, mask, Mat(), Point(-1, -1), 1);
    // dilate(mask, mask, Mat(), Point(-1, -1), 1); 
    // imshow("binary", mask);
    
    // namedWindow("mask", WINDOW_AUTOSIZE);
    // createTrackbar("Lower H", "mask", &lower_H, 255, on_HLS_change);
    // createTrackbar("Upper H", "mask", &upper_H, 255, on_HLS_change);
    // createTrackbar("Lower L", "mask", &lower_L, 255, on_HLS_change);
    // createTrackbar("Upper L", "mask", &upper_L, 255, on_HLS_change);
    // createTrackbar("Lower S", "mask", &lower_S, 255, on_HLS_change);
    // createTrackbar("Upper S", "mask", &upper_S, 255, on_HLS_change);
    // on_HLS_change(0, 0);
    // 트랙바 안쓰고 inRange
    inRange(frame2, Scalar(0, 170, 0), Scalar(255, 255, 255), mask);
    imshow("mask", mask);
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
    int minarea = 32000;
    int nowarea = 0;
    for (int i = 0; i < contours.size(); i++)
    {
        double a = contourArea(contours[i]);
        // 모든 외각선의 사이즈 출력
        // cout << "contour idx = " << i << " " << "size = " << a << endl;
        if (a > 14000 & a < 31000)
        {
            nowarea = a;
            if(a < minarea)
            {
                minarea = a;
                lrgctridx = i;
            }
            
        }
    }
    Mat drawing = Mat::zeros( mask2.size(), CV_8UC3 );

    vector<Point> hull;
    convexHull(Mat(contours[lrgctridx]), hull, false);

    vector<vector<Point>> fake_hull;
    fake_hull.push_back(hull);


    // 모든 외각선 그리기
    // for(int idx = 0; idx >= 0; idx = hierarchy[idx][0])
    // {
    //     drawContours( drawing, contours, idx, color, 2, LINE_8, hierarchy);
    // }
    // 특정한 외각선만 그리기
    drawContours( drawing, contours, lrgctridx, color, 2, LINE_8, hierarchy);
    drawContours(drawing, fake_hull, 0, color, 2, LINE_8);
    

    int max_x = hull[0].x;
    int max_x_num = 0;

    int max_y = hull[0].y;
    int max_y_num = 0;

    for(int i = 1; i < hull.size(); i++)
    {
        if(hull[i].x >= max_x)
        {
            max_x = hull[i].x;
            max_x_num = i;
        }
        if(hull[i].y >= max_y)
        {
            max_y = hull[i].y;
            max_y_num = i;
        }
    }

    int mean_x = int((hull[max_x_num].x+hull[max_y_num].x) / 2);
    int mean_y = int((hull[max_x_num].y+hull[max_y_num].y) / 2);

    circle(drawing, hull[max_x_num], 10, Scalar(255, 0, 0), -1);
    circle(drawing, hull[max_y_num], 10, Scalar(0, 0, 255), -1);
    circle(drawing, Point(mean_x, mean_y), 10, Scalar(0, 255, 0), -1);

    // cout << hull[max_x_num] << endl;

    imshow("drawing", drawing);

    Point final(mean_x, mean_y);
    return final;

    // 무게중심점
    // Moments m = moments(contours[lrgctridx], true);
    // Point p(m.m10/m.m00, m.m01/m.m00);
    // cout << "centroid : "<< p << endl;
    // circle(drawing, p, 10, Scalar(255, 255, 255), -1);
    // imshow( "contour", drawing );

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
    waitKey(delay);
}

// void parking_levle3()
// {

// }

Mat preprocessing_forth_level(Mat img)
{

	// imshow("img_resize10",img_resize10);
	// cout<<"!!!!!!!!!!!!!!!!!!!!!111"<<endl;
	img_color = img.clone();
	img_resize_new = img.clone();
	// img_resize2 = img_resize.clone();

	// namedWindow("img_color", WINDOW_AUTOSIZE); //window for output mask
	// setMouseCallback("img_color", mouse_callback);
	// imshow("img_color", img);
	// // waitKey();

	Mat HSV_image;
	cvtColor(img, HSV_image, COLOR_BGR2HSV);

	Scalar lower_white = Scalar(0, 0, 170);
	Scalar upper_white = Scalar(255, 100, 255);

	Mat white_image;
	inRange(HSV_image, lower_white, upper_white, white_image);
	// imshow("white_image", white_image);
	// cout<<"^^^^^^^^^^^^^^^^^^^"<<endl;
	Mat img_warp11;
	img_warp11 = bird_eyes_view_for_front_camara(white_image);
	// imshow("warp_img", img_warp11);

	Mat img_warp111 = img_warp11.clone();

	Mat dx, dy;
	Sobel(img_warp11, dx, CV_32FC1, 1, 0);
	Sobel(img_warp11, dy, CV_32FC1, 0, 1);

	Mat fmag, mag;
	magnitude(dx, dy, fmag);
	fmag.convertTo(mag, CV_8UC1);
	Mat edge = mag > 130;
	//imshow("edge", edge);

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

	Mat warp_inv;
	warp_inv = bird_eyes_view_inverse_for_front_camara(dst_4);
	// imshow("warp_inv", warp_inv);

	Mat img_integral;
	cv::integral(img_warp111, img_integral);

	Mat img_mask;
	img_mask = mask_filter(img_integral, 5, 5, 95);
	//imshow("img_mask", img_mask);

	Mat warp_inv11;
	warp_inv11 = bird_eyes_view_inverse_for_front_camara(img_mask);
	// imshow("warp_inv11", warp_inv11);

	Mat final2;
	cv::resize(warp_inv11, final2, cv::Size(640, 480), 0, 0);
	// imshow("img_resize",final2);

	Mat final3;
	cv::resize(warp_inv, final3, cv::Size(640, 480), 0, 0);
	// imshow("img_resize",final3);
	// cout<<"final3.size: "<<final3.size()<<endl;
	// cout<<"final3.type: "<<final3.type()<<endl;
	Mat final4 = final2 + final3;
	// imshow("final4",final4);

	Mat final10;
	addWeighted(img_resize_new, 0.5, final4, 0.5, 0, final10);

	int count = 100;

	if (isStop == 5)
	{
		count = 5;
		//putText(final10, "7M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
	}

	else if (isStop == 4)
	{
		count = 4;
		// FinalLevel = 1;
		//putText(final10, "6M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
	}

	else if (isStop == 3)
	{
		count = 3;
		// FinalLevel = 1;
		//putText(final10, "5M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
	}

	else if (isStop == 2)
	{
		count = 2;
		FinalLevel = 1;
		//putText(final10, "4M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
	}

	else if (isStop == 1)
	{
		count = 1;
		FinalLevel = 1;
		//putText(final10, "STOP!!", Point(380, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
	}
	// else if (isStop == 10)
	// {
	// 	count = 100;

	// 	//putText(final10, "GO!!", Point(380, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
	// }
	std_msgs::Float64 stopline_msg;
	stopline_msg.data = count;
	cout << "stopline" << endl;
	ROS_INFO("%f", stopline_msg.data);
	cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1" << endl; // data 메시지를표시한다
	stopline_pub.publish(stopline_msg);									// 메시지를발행한다

	// imshow("final10", final10);

	return final10;
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
    image_transport::Subscriber sub_image = it.subscribe("/camera/rgb/image_raw", 100, imageCallback);
    image_transport::Publisher pub = it.advertise("camera/image_raw", 10);
    sensor_msgs::ImagePtr msg;
    ros::Rate loop_rate(50);
    printf("Waiting for ---/camera/stopline/image_raw---\n");

    VideoCapture cap("/home/kroad/catkin_ws/src/parking/src/park2.mp4");

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

    Mat frame, frame3;

    while(ros::ok())
    {
        if (!cap.isOpened())
        {
            cerr << "finish!\n" << endl;
        }

        cap >> frame;

        Mat frame_re;
        resize(frame, frame_re, Size(640, 480), 0, 0, INTER_CUBIC);

        Mat img_resize6;
        cv::resize(frame_for_front_camara, img_resize6, cv::Size(640, 480), 0, 0);

        std_msgs::Float64 encoder_mode_msg;

        double fps = cap.get(CAP_PROP_FPS);
        cout << "FPS: " << fps << endl;
        int delay = cvRound(1000 / fps);


        if (parking1 == false)
        {
            rec_center = parking_level1(frame_re, delay);
        }  


        else if(parking1 == true && third_level == false)
        {

            Rect2d bbox(Point(rec_center.x-25, rec_center.y - 15), Point(rec_center.x + 25, rec_center.y + 10));
            // rectangle(frame_re, bbox, Scalar( 255, 0, 0 ), 2, 1 );

            imshow("Tracking", frame_re);
            tracker->init(frame_re, bbox);

            double timer = (double)getTickCount();
            bool ok = tracker->update(frame_re, bbox);

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

            imshow("Tracking", frame_re);
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

            // contours
            vector<vector<Point>> contours;
            vector<Vec4i> hierarchy;
            findContours(src_HLS, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
            Scalar color(rand() & 255, rand() & 255, rand() & 255);
            // int lrgctridx = 0;
            // int maxarea = 0;
            // int nowarea = 0;
            // for (int i = 0; i < contours.size(); i++)
            // {
            //     double a = contourArea(contours[i]);
            //     // 모든 외각선의 사이즈 출력
            //     cout << "contour idx = " << i << " " << "size = " << a << endl;
            //     if (a > 14000 & a < 31000)
            //     {
            //         nowarea = a;
            //         if(a > maxarea)
            //         {
            //             maxarea = a;
            //             lrgctridx = i;
            //         }
                    
            //     }
            // }

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

            // // 캐니엣지
            // // Mat edge;
            // // Canny(src_HLS, edge, 50, 150);

            // //허프라인으로 선 감지

            // // 방법1
            // // Mat line_detect_G;
            // // cvtColor(frame, line_detect_G, COLOR_BGR2GRAY);

            // // vector<Vec2f> lines;
            // // HoughLines(edge, lines, 1, CV_PI / 180, 40);

            // // Mat dst;
            // // cvtColor(edge, dst, COLOR_GRAY2BGR);

            // // for(size_t i = 0; i < lines.size(); i++)
            // // {
            // //     float r = lines[i][0], t = lines[i][1];
            // //     double cos_t = cos(t), sin_t = sin(t);
            // //     double x0 = r * cos_t, y0 = r * sin_t;
            // //     double alpha = 1000;

            // //     Point pt1(cvRound(x0 + alpha * (-sin_t)), cvRound(y0 + alpha * cos_t));
            // //     Point pt2(cvRound(x0 - alpha * (-sin_t)), cvRound(y0 - alpha * cos_t));
            // //     line(dst, pt1, pt2, Scalar(0, 0, 255), 2, LINE_AA);
            // // }

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

            theta_sum /= theta_list.size();

            final_theta = theta_sum;

            std_msgs::Float64 parking_level2_msg;

            parking_level2_msg.data = theta_sum;
            parking_level2_pub.publish(parking_level2_msg);

            if(final_theta <= 50) third_level = true;

            imshow("line_detect", line_detect);
            imshow("dst", drawing_G);

            Mat edge_resize;
            resize(drawing_G, edge_resize, Size(cols, rows), 0, 0, INTER_CUBIC);
            imshow("edge_resize", edge_resize);

            Mat edge_final = Mat::zeros(480, 640, CV_8UC3);
            
            // 최종 영상에 허프라인 결과 합성
            Mat final_roi = final(bbox);
            addWeighted(final_roi, 0.3, edge_resize, 1, 0, final_roi);

            
            // final = final + edge;

            putText(final, "theta_live : " + SSTR(float(final_theta)), Point(300,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);

            imshow("final", final);

            waitKey(delay);
        }

       else if (third_level == true)
			{
				encoder = 2;
				cout << "44444444444444444444444" << endl;
				frame3 = preprocessing_forth_level(img_resize6);
				// imshow("frame3",frame3);
				Mat final7;
				addWeighted(img_resize1000, 0.5, frame3, 0.5, 0, final7);

				if (isStop == 5)
				{
					// count = 5;
					putText(final7, "5M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
				}

				else if (isStop == 4)
				{
					// count = 4;
					putText(final7, "4M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
				}

				else if (isStop == 3)
				{
					// count = 3;
					putText(final7, "3M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
				}

				else if (isStop == 2)
				{
					// count = 2;
					putText(final7, "2M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
				}

				else if (isStop == 1)
				{
					// count = 1;
					putText(final7, "1M!!", Point(380, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
					// isStop=10;
				}
				else if (isStop == 10)
				{
					// count = 10;
					putText(final7, "GO!", Point(380, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
				}
				imshow("final7", final7);

				if (FinalLevel == 1 || enc_fail == true)
				{
					third_level = false;
					cout << "enc_fail : " << enc_fail << endl;
					cout << "FinalLevel : " << FinalLevel << endl;
					encoder = 1;
				}
			}

			// // cout << frame2.type() << img_resize1.type() << endl;
			// // cout << frame2.size() << img_resize1.size() << endl;
			// // imshow("frame2", frame2);
			encoder_mode_msg.data = encoder;
			encoder_pub.publish(encoder_mode_msg);
			// ROS_INFO("%", encoder_mode_msg.data);
			cout << "encoder_mode_msg.data: " << encoder << endl;

			if (waitKey(delay) == 27)
			{
				break;
				printf("end");
			}

        cout << rec_center << endl;



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
	// imshow("image1", image1);
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
			else if (y < 240)
			{
				cout << "stop line distance : 4M\n"
					 << endl;
				isStop = 4;
			}
			else if (y < 340)
			{
				cout << "stop line distance : 3M\n"
					 << endl;
				isStop = 3;
			}
			else if (y < 440)
			{
				cout << "stop line distance : 2M\n"
					 << endl;
				isStop = 2;
			}
			else if (y < 470)
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