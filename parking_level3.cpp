	#include "opencv2/opencv.hpp"
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

using namespace std;
using namespace cv::ml;
using namespace cv;

Mat preprocessing(Mat img);
Mat bird_eyes_view(Mat img);
Mat warp_matrix_inv;
void on_mouse(int event, int x, int y, int flags, void*);
Point ptOld1;
Mat bird_eyes_view_inverse(Mat img);
int H, S, V;
Mat img_color;
Mat img_hsv;
Mat OutputImage;
int isStop = 0;
bool parking1_ =false;
void mouse_callback(int event, int x, int y, int flags, void *param);
int a1 = 0, a2 = 0, a3 = 0, a4 = 0, a5 = 0;
int x11 = 0, x22 = 0, x33 = 0, x44=0, y11 = 0, y22 = 0, y33 = 0, y44=0;
const double PI = 3.1415926535897932384626433832795028841971693993751058209;


void parking_cb(const std_msgs::Bool::ConstPtr& msgs)
{
	parking1_ = msgs->data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "parking_level3");
	ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image_raw", 10);
    image_transport::Publisher pub1 = it.advertise("camera/image_final", 10);
	ros::Publisher parking_level3_pub = nh.advertise<std_msgs::Float64>("ParkingAngle_RIGHT", 10);
	ros::Subscriber Parking_level1_sub = nh.subscribe("parking1", 10, parking_cb);
	std_msgs::Float64 ParkingAngle_RIGHT_msg;

    ros::Rate loop_rate(50);
    sensor_msgs::ImagePtr msg;
    sensor_msgs::ImagePtr msg1;
	
 		// VideoCapture cap1("/home/usera/catkin_ws/src/record_video/video/주차(세로선).mp4");
		// VideoCapture cap1(2);
		VideoCapture cap1("/home/usera/바탕화면/parking/src/parking_kcity_right.mp4");

		if (!cap1.isOpened())
			{
				cerr << "video load fail!!\n"
					<< endl;
			}

		double fps = cap1.get(CAP_PROP_FPS);
		cout << "FPS: " << fps << endl;
		int delay = cvRound(1000 / fps);

		Mat frame1,frame2;


		while (ros::ok())
		{
			// if(parking1_)
			// {	
				waitKey();
				cap1 >> frame1;
				// imshow("frame1",frame1);
				msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame1).toImageMsg();
				pub.publish(msg);

				frame2 = preprocessing(frame1);
				// imshow("frame2",frame2);

				Mat labels4, stats4, centroids4;
				int cnt4 = connectedComponentsWithStats(frame2, labels4, stats4, centroids4);

				Mat dst_4;
				cvtColor(frame2, dst_4, COLOR_GRAY2BGR);

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

					if (p[4] > 4000 && p[4] < 30000)   //5000     // 15000 수정
					{
						if ((theta_live > 50 && theta_live < 88) || (theta_live < -70 && theta_live > -89))
						{
							// cout << "*************************************8" << endl;
							rectangle(dst_4, Rect(p[0], p[1], p[2], p[3]), Scalar(0, 0, 255), 2);
							// cout << "stats1 : " << p[4] << endl;
							ParkingAngle_RIGHT_msg.data = theta_live;
							parking_level3_pub.publish(ParkingAngle_RIGHT_msg);
							// ROS_INFO("%f", ParkingAngle_RIGHT_msg.data);
							cout<<theta_live<<"도 꺾으세요!!!!!"<<endl;
						}
					}
				}
				//   imshow("dst_4", dst_4);

				Mat warp_inv;
				warp_inv = bird_eyes_view_inverse(dst_4);
				imshow("warp_inv", warp_inv);
				
				Mat img_resize1; 
				cv::resize(frame1, img_resize1, cv::Size(640, 480), 0, 0);
				imshow("img_resize",img_resize1);
			
				Mat final1;
				addWeighted(img_resize1, 0.5, warp_inv, 0.5, 0, final1);
				imshow("final1", final1);

				msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", final1).toImageMsg();
				pub1.publish(msg1);
				
				ros::spinOnce();
				loop_rate.sleep();
				// if (waitKey(delay) == 27) 
				// 	break;
			// }
		}

	return 0;
}

Mat preprocessing(Mat img)
{
	Mat img_resize;
	cv::resize(img, img_resize, cv::Size(640, 480), 0, 0);
	//imshow("img_resize", img_resize);

	//namedWindow("white_image@@@");
	//setMouseCallback("white_image@@@", on_mouse);
	//imshow("white_image@@@", img_resize);
	//waitKey();
	
	img_color = img_resize.clone();

	namedWindow("img_color", WINDOW_AUTOSIZE); //window for output mask
	setMouseCallback("img_color", mouse_callback);
	// imshow("img_color", img_resize);
	// waitKey();
	
	Mat HSV_image;
	cvtColor(img_resize, HSV_image, COLOR_BGR2HSV);

	// Scalar lower_white = Scalar(H, S, V);
	// Scalar upper_white = Scalar(255, 255, 255);

	Scalar lower_white = Scalar(0, 0, 170);     
		Scalar upper_white = Scalar(255, 100, 255);

	Mat white_image;
	inRange(HSV_image, lower_white, upper_white, white_image);
	imshow("white_image", white_image);

	Mat img_warp;
	img_warp = bird_eyes_view(white_image);
	imshow("warp_img", img_warp);	

	Mat dx, dy;
	Sobel(img_warp, dx, CV_32FC1, 1, 0);
	Sobel(img_warp, dy, CV_32FC1, 0, 1);

	Mat fmag, mag;
	magnitude(dx, dy, fmag);
	fmag.convertTo(mag, CV_8UC1);
	Mat edge = mag > 130;
	imshow("edge", edge);

	vector<Vec4i> lines;
	HoughLinesP(edge, lines, 1, CV_PI / 180, 160, 50, 5);

	Mat dst;
	cvtColor(edge, dst, COLOR_GRAY2BGR);
	Mat dst4(Size(640, 480), CV_8UC1);

	for (Vec4i l : lines)
	{
		//line(dst,Point(l[0],l[1]),Point(l[2],l[3]),Scalar(0,0,255),2,LINE_AA);
		//cout << "lines.size: " << lines.size() << endl;
		float dx = l[2] - l[0], dy = l[3] - l[1];
		float k = dy / dx;
		float radian = atan(k);
		float theta = (radian * 180) / PI;
		float d = l[1];
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

		if ((theta > 50 && theta < 88) || (theta < -70 && theta > -89))    //50   -70
		{
			line(dst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2, LINE_AA);
			//cout << "Point(l[0], l[1]): " << Point(l[0], l[1]) << " Point(l[2], l[3]): " << Point(l[2], l[3]) << endl;
			line(dst4, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), 15, LINE_AA);
			//cout << "theta_original: " << theta << endl;
		}
		
		// else if (isStop == 0)
		// {
		// 	cout << "nothing" << endl;
		// }
	}

	imshow("dst", dst);
	imshow("dst4", dst4);

	int num = (int)(dst4.total()*0.1);
	for (int i = 0; i < num; i++)
	{
		int x44 = rand() % dst4.cols;
		int y44 = rand() % dst4.rows;
		dst4.at<uchar>(y44, x44) = (i % 2) * 255;	
	}

	Mat dst_final_blur4;
	medianBlur(dst4, dst_final_blur4, 29);
	imshow("dst_final_blur4",dst_final_blur4);

	

	return dst_final_blur4;
}
	
Mat bird_eyes_view(Mat img)
{
	int width = img.cols;
	int height = img.rows;

	width = img.cols;
	height = img.rows;
	Mat warp_matrix;
	Point2f warp_src_point[4];
	Point2f warp_dst_point[4];

	warp_src_point[0].x = 15;
	warp_src_point[0].y = 350;
	warp_src_point[1].x = 470;
	warp_src_point[1].y = 350;
	warp_src_point[2].x = 15;
	warp_src_point[2].y = 90;
	warp_src_point[3].x = 470;
	warp_src_point[3].y = 90;
	
	warp_dst_point[0].x = 150;
	warp_dst_point[0].y = height * 0.8;
	warp_dst_point[1].x = width - 150;
	warp_dst_point[1].y = height * 0.8;
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


void on_mouse(int event, int x, int y, int flags, void*)
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


Mat bird_eyes_view_inverse(Mat img)
{
	int width = img.cols;
	int height = img.rows;

	width = img.cols;
	height = img.rows;
	Mat warp_matrix;
	Point2f warp_src_point[4];
	Point2f warp_dst_point[4];

	warp_src_point[0].x = 15;
	warp_src_point[0].y = 350;
	warp_src_point[1].x = 470;
	warp_src_point[1].y = 350;
	warp_src_point[2].x = 15;
	warp_src_point[2].y = 90;
	warp_src_point[3].x = 470;
	warp_src_point[3].y = 90;

	warp_dst_point[0].x = 150;
	warp_dst_point[0].y = height * 0.8;
	warp_dst_point[1].x = width - 150;
	warp_dst_point[1].y = height * 0.8;
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

void mouse_callback(int event, int x, int y, int flags, void *param)
{
	if (event == EVENT_LBUTTONDBLCLK)
	{
		Vec3b color_pixel = img_color.at<Vec3b>(y, x);

		Mat bgr_color = Mat(1, 1, CV_8UC3, color_pixel);


		Mat hsv_color;
		cvtColor(bgr_color, hsv_color, COLOR_BGR2HSV);

		H = hsv_color.at<Vec3b>(0, 0)[0];
		S = hsv_color.at<Vec3b>(0, 0)[1];
		V = hsv_color.at<Vec3b>(0, 0)[2];

		cout << "H= " << H << endl;
		cout << "S= " << S << endl;
		cout << "V = " << V << "\n" << endl;

		H = H - 200;
		S = S - 50;
		V = V - 70;

		if (H < 0)
			H = 0;

		if (S < 0)
			S = 0;

		if (V < 0)
			V = 0;
	}
}