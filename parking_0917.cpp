#include "StereoVision.h"
#include "StereoVision.cpp"

// ------------CAMERA NUMBER 입력--------------
#define LEFT_CAM 2
#define RIGHT_CAM 0


using namespace cv;
using namespace std;

void parking_cb_1(const std_msgs::Bool::ConstPtr &msgs);
void mission_cb(const std_msgs::Int16::ConstPtr& msgs);

int array_count = 0;

float baseline = 23; //카메라 사이 거리
float focal_pixels = 800; // size(1280, 720) 일때 focal_pixels
float alpha = 10.5;	//alpha = 카메라 머리 숙인 각도
float beta = 43.8496; 	//beta = erp 헤딩으로부터 카메라 각도
float gps_for_camera_x = 30; //cm
float gps_for_camera_z = -50; //cm

int target_x = 135;
int target_z = 135;
int mission_flag = 0;


Mat img_color;
Mat img_color_2;
int H, S, V;
void mouse_callback(int event, int x, int y, int flags, void *param);
void mouse_callback_2(int event, int x, int y, int flags, void *param);
void on_mouse(int event, int x, int y, int flags, void *);

bool Lidar_Stop = false;
bool finish_park = false;

bool impulse = false;

float sum_array[6] ={0,0,0,0,0,0};

ros::Subscriber Parking_level1_sub;
ros::Subscriber mission_sub;


Point ptOld1;

float camera_values();

int main(int argc, char **argv) 
{   
	ros::init(argc, argv, "parking_publisher");

	string left_cam_index = "/dev/video" + to_string(LEFT_CAM);
	string right_cam_index = "/dev/video" + to_string(RIGHT_CAM);

    VideoCapture capLeft(left_cam_index);
    VideoCapture capRight(right_cam_index);
	// VideoCapture capLeft("/home/korus/catkin_ws/src/record_video/stereo_video/left1.mp4");
    // VideoCapture capRight("/home/korus/catkin_ws/src/record_video/stereo_video/right1.mp4");
    
	if (!capLeft.isOpened()) {
        cout << "Cannot Open Left Camera" << endl;
    }
    if (!capRight.isOpened()) {
        cout << "Cannot Open Right Camera" << endl;
    }

	// Korus ver. topic
	ros::NodeHandle nh;
	ros::Publisher center_XZ_pub = nh.advertise<std_msgs::Float64MultiArray>("Vision/diag_points", 10);
	Parking_level1_sub = nh.subscribe("/LiDAR/Park_Stop", 10, parking_cb_1); //lidar parkingstop
    mission_sub = nh.subscribe("/Planning/mission", 10, mission_cb);

    capLeft.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    capLeft.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    capRight.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    capRight.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    StereoVision stereovision(baseline, focal_pixels);

	Mat leftFrame, rightFrame;
    Mat leftMask, rightMask;
	
    Point leftCircle, rightCircle; // using yellow ball
    Point2f top_XZ, mid_XZ, bottom_XZ; // using parking

    while (ros::ok()) 
    {
		capLeft.read(leftFrame);
		capRight.read(rightFrame);
		resize(leftFrame, leftFrame, Size(1280, 720));
		resize(rightFrame, rightFrame, Size(1280, 720));

		if((mission_flag == 10)||(mission_flag == 6))
		{
			// system("clear");
			// cout << "---------------------------------------------" << endl;
			// cout << "Lidar_Stop : " << Lidar_Stop << "  finish_park : " << finish_park << endl;

			// Calibration of the frames (640,480)
			// leftFrame = stereovision.undistortFrame(leftFrame);
			// rightFrame = stereovision.undistortFrame(rightFrame);
			
			// rectangle(leftFrame, Rect(0, 0, 1280, 100), Scalar(0, 0, 0), -1); //상단 for koreatech
			// rectangle(rightFrame, Rect(0, 0, 1280, 100), Scalar(0, 0, 0), -1); //상단 for koreatech
			// rectangle(leftFrame, Rect(0, 0, 1280, ting), Scalar(0, 0, 0), -1); //for k-citys
			// rectangle(rightFrame, Rect(0, 0, 1280, 200), Scalar(0, 0, 0), -1); //for k-citys

			imshow("Left Frame", leftFrame);
			imshow("Right Frame", rightFrame);

			leftMask = stereovision.add_HSV_filter(leftFrame, 0);
			rightMask = stereovision.add_HSV_filter(rightFrame, 1);

			// ==================[ using mouse_callback ]=====================
			// img_color = leftFrame.clone();
			// img_color_2 = rightFrame.clone();
			// setMouseCallback("Left Frame", mouse_callback);
			// setMouseCallback("Right Frame", mouse_callback_2);

			// setMouseCallback("Left Frame",on_mouse);
			// setMouseCallback("Right Frame",on_mouse);

			if((Lidar_Stop == false) && (finish_park == false))
			{
				// cout << " Wating Lidar Stop" << endl;
				imshow("Left_mask", leftMask);
				imshow("Right_mask", rightMask);
			}
			else if((Lidar_Stop == true) && (finish_park == false))
			{	
				if(impulse == false)
				{
					cout << "impulse !!" << endl;
					ros::Duration(1.5).sleep();
					impulse = true;
				}
				imshow("Left_mask", leftMask);
				imshow("Right_mask", rightMask);
				double left_array[6] = {0,0,0,0,0,0};
				double right_array[6] = {0,0,0,0,0,0};
				double array[6] = {0,0,0,0,0,0};
				double pub_array[6] = {0,0,0,0,0,0};

				double *ptr_left = stereovision.find_center(leftMask, left_array, 0);
				double *ptr_right = stereovision.find_center(rightMask, right_array, 1);

				// If no ball is detected in one of the cameras - show the text "tracking lost"
				if (left_array[0] && right_array[0])
				{
					bottom_XZ = stereovision.find_XZ({left_array[0],left_array[1]}, {right_array[0],right_array[1]}, leftFrame, rightFrame, alpha, beta);
					// mid_XZ = stereovision.find_XZ({left_array[2],left_array[3]}, {right_array[2],right_array[3]}, leftFrame, rightFrame, alpha, beta);
					top_XZ = stereovision.find_XZ({left_array[4],left_array[5]}, {right_array[4],right_array[5]}, leftFrame, rightFrame, alpha, beta);
					mid_XZ.x = (bottom_XZ.x + top_XZ.x)/2.0;
					mid_XZ.y = (bottom_XZ.y + top_XZ.y)/2.0;

					// cout << "bottom_XZ : " << bottom_XZ << endl;
					// cout << "mid_XZ : " << mid_XZ << endl;
					// cout << "top_XZ : " << top_XZ << endl;

					array[0]= (bottom_XZ.x + gps_for_camera_z)/100.00;
					array[1]= -(bottom_XZ.y + gps_for_camera_x)/100.00;
					array[2]= (mid_XZ.x + gps_for_camera_z)/100.00;
					array[3]= -(mid_XZ.y + gps_for_camera_x)/100.00;
					array[4]= (top_XZ.x + gps_for_camera_z)/100.00;
					array[5]= -(top_XZ.y + gps_for_camera_x)/100.00;
					
					for(int i=0; i<6; i++)
					{
						sum_array[i] = sum_array[i] + array[i];
					}				

					array_count++;
					cout << "array_count : " << array_count << endl;
					if(array_count == 20)
					{
						cout << "!!!!!!!!!!!!!!!!!!" << endl;
						std_msgs::Float64MultiArray center_XZ_msg;
						center_XZ_msg.data.clear();

						for(int i=0; i<6; i++)
						{
							pub_array[i] = sum_array[i]/(double)array_count;
							center_XZ_msg.data.push_back(pub_array[i]);
							printf("pub_array[%d] : %f\n", i, pub_array[i]);
						}
						center_XZ_pub.publish(center_XZ_msg);
						finish_park = true;
					}
				}
			}
			else if((Lidar_Stop == true) && (finish_park == true))
			{
				// cout << " Finish !!!! " << endl;
			}
		}

		if (waitKey(10) == 27)
		{
			break;
			printf("end");
		}
		
        // capLeft.release();
        // capRight.release();
        ros::spinOnce();
	    // loop_rate.sleep();
    }
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

// float camera_values()
// {
//     cv::Mat K(3,3,CV_64F);

//     K = (Mat_<_Float64>(3, 3) << 538.39128993937,   0.,   308.049009633327, 0.,  539.777910345317,  248.065244763797, 0., 0., 1.);

//     cout << "K : " << K << endl;


//     cv::Size imageSize(640,480);
//     double apertureWidth = 0;
//     double apertureHeight = 0;
//     double fieldOfViewX;
//     double fieldOfViewY;
//     double focalLength2;
//     cv::Point2d principalPoint;
//     double aspectRatio;
//     cv::calibrationMatrixValues(K, imageSize, apertureWidth, apertureHeight, fieldOfViewX, fieldOfViewY, focalLength2, principalPoint, aspectRatio);

    
//     cout << fieldOfViewX << endl;

//     return (float)focalLength2;
// }

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

void parking_cb_1(const std_msgs::Bool::ConstPtr &msgs)
{
	Lidar_Stop = msgs->data;
}

void mission_cb(const std_msgs::Int16::ConstPtr& msgs)
{
	mission_flag = msgs->data;
}

// void parking_cb_2(const std_msgs::Bool::ConstPtr &msgs)
// {
// 	parking_level_2 = msgs->data;
// }

// void parking_cb_3(const std_msgs::Bool::ConstPtr &msgs)
// {
// 	parking_level_3 = msgs->data;
// }

// void imageCallback(const sensor_msgs::ImageConstPtr &msg)
// {
// 	// printf("이미지 수신중\n");
// 	Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
// 	front_camera = image.clone();
// 	cv::waitKey(1);
// 	callback = true;
// 	// imshow("image", image);
// }

// Mat find_stopline(Mat &img)
// {
// 	Mat img_clone = img.clone();
	
// 	Mat HSV_image;
// 	cvtColor(img, HSV_image, COLOR_BGR2HSV);

// 	Scalar lower_white = Scalar(0, 0, 180);
// 	Scalar upper_white = Scalar(255, 100, 255);

// 	Mat white_image;
// 	inRange(HSV_image, lower_white, upper_white, white_image);
// 	// imshow("white_image", white_image);

// 	Mat warp_image;
// 	warp_image = bird_eyes_view_for_front_camara(white_image, 0);
// 	imshow("warp_img", warp_image);

// 	Mat warp_image_clone = warp_image.clone();

// 	Mat integtal_image;
// 	cv::integral(warp_image_clone, integtal_image);

// 	Mat mask_image;
// 	mask_image = mask_filter(integtal_image, 5, 5, 90);
// 	//imshow("mask_image", mask_image);

// 	Mat warp_inverse_image;
// 	warp_inverse_image = bird_eyes_view_for_front_camara(mask_image, 1);
// 	// imshow("warp_inverse_image", warp_inverse_image);

// 	resize(warp_inverse_image, warp_inverse_image, Size(640, 480), 0, 0);
// 	// imshow("img_resize",warp_inverse_image);

// 	Mat final_image;
// 	addWeighted(img_clone, 0.5, warp_inverse_image, 0.5, 0, final_image);

// 	int count = 100;

// 	if (isStop == 5)
// 	{
// 		count = 5;
// 		putText(final_image, "5M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
// 	}
// 	else if (isStop == 4)
// 	{
// 		count = 4;
// 		putText(final_image, "4M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
// 	}
// 	else if (isStop == 3)
// 	{
// 		count = 1;
// 		off ++;
// 		putText(final_image, "3M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
// 	}
// 	else if (isStop == 2)
// 	{
// 		count = 1;
// 		off ++;
// 		putText(final_image, "2M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
// 	}
// 	else if (isStop == 1)
// 	{
// 		count = 1;
// 		off ++;
// 		putText(final_image, "1M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
// 	}
// 	else if (isStop == 10)
// 	{
// 		putText(final_image, "GO!", Point(380, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
// 	}

// 	if(off > 5)
// 	{
// 		FinalLevel = true;
// 		count = 10;
// 	}

// 	std_msgs::Float64 stopline_msg;
// 	stopline_msg.data = count;

// 	cout << "stopline" << endl;
// 	ROS_INFO("%f", stopline_msg.data);
	
// 	stopline_pub.publish(stopline_msg);						// 메시지를발행한다
// 	// imshow("final_image", final_image);

// 	return final_image;
// }

// Mat mask_filter(Mat &img, int _mask_w, int _mask_h, int thresh)
// {
// 	int height = img.rows;
// 	int width = img.cols;
// 	Mat img_maskfilter;
// 	img_maskfilter = Mat::zeros(height, width, CV_8UC1);
// 	Mat img_stop;
// 	img_stop = Mat::zeros(height, width, CV_8UC3);
// 	float mask[3];
// 	isStop = 10;

// 	// putText(img_stop, "7M", Point(0, 40), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
// 	// line(img_stop, Point(3, 40), Point(600, 40), Scalar(0, 0, 255));
// 	// putText(img_stop, "6M", Point(0, 140), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255));
// 	// line(img_stop, Point(3, 140), Point(600, 140), Scalar(255, 0, 255));
// 	// putText(img_stop, "5M", Point(0, 240), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255));
// 	// line(img_stop, Point(3, 240), Point(600, 240), Scalar(0, 255, 255));
// 	// putText(img_stop, "4M", Point(0, 340), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 0));
// 	// line(img_stop, Point(3, 340), Point(600, 340), Scalar(255, 255, 0));
// 	// putText(img_stop, "3M", Point(0, 440), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
// 	// line(img_stop, Point(3, 440), Point(600, 440), Scalar(0, 0, 255));

// 	uint *image = (uint *)img.data;
// 	uchar *score_data = (uchar *)img_maskfilter.data;
// 	int mask_w = _mask_w, mask_h = _mask_h;

// 	int sy = 0;

// 	int roi_w = 100;
// 	int histo = 0;
// 	for (int y = 20; y < height - 17; y++)
// 	{
// 		histo = 0;
// 		for (int x = int(width / 2) - roi_w; x <= int(width / 2) + roi_w; x++)
// 		{
// 			for (int i = 0; i < 3; i++)
// 			{
// 				sy = y + (2 * mask_h + 1) * (i - 1);
// 				int dx, cx, bx, ax;
// 				int dy, cy, by, ay;
// 				dy = sy + mask_h;
// 				dx = x + mask_w;
// 				cy = sy - mask_h - 1;
// 				cx = x + mask_w;
// 				by = sy + mask_h;
// 				bx = x - mask_w - 1;
// 				ay = sy - mask_h - 1;
// 				ax = x - mask_w - 1;
// 				mask[i] = image[(dy)*width + dx] - image[(cy)*width + cx] - image[(by)*width + bx] + image[(ay)*width + ax];
// 			}

// 			float sum = ((mask[1] - mask[0]) + (mask[1] - mask[2])) / 2;
// 			if (sum > 8000)
// 			{
// 				score_data[width * y + x] = 255;
// 				histo++;
// 			}
// 		}

// 		if (histo > thresh)
// 		{
// 			line(img_stop, Point(int(width / 2) - roi_w, y), Point(int(width / 2) + roi_w, y), Scalar(255, 0, 0), 30);
// 			if (y < 260) //140
// 			{
// 				cout << "stop line distance : 5M"<< endl << endl;
// 				isStop = 5;
// 			}
// 			else if (y < 300) //230
// 			{
// 				cout << "stop line distance : 4M" << endl << endl;
// 				isStop = 4;
// 			}
// 			else if (y < 340) //267
// 			{
// 				cout << "stop line distance : 3M" << endl << endl;

// 				isStop = 3;
// 			}
// 			else if (y < 400)//330
// 			{
// 				cout << "stop line distance : 2M" << endl << endl;
// 				isStop = 2;
// 			}
// 			else
// 			{
// 				cout << "stop line distance : 1M" << endl << endl;
// 				isStop = 1;
// 			}
// 			break;
// 		}
// 	}
// 	// imshow("img_stop",img_stop);
// 	return img_stop;
// }

// Mat bird_eyes_view_for_front_camara(Mat &img, int num)
// {
// 	int width = img.cols;
// 	int height = img.rows;

// 	width = img.cols;
// 	height = img.rows;
// 	Mat warp_matrix;
//     Mat warp_matrix_inv;

// 	Point2f warp_src_point[4];
// 	Point2f warp_dst_point[4];

// 	// warp_src_point[0].x = 40;
// 	// warp_src_point[0].y = 480;
// 	// warp_src_point[1].x = 600;
// 	// warp_src_point[1].y = 480;
// 	// warp_src_point[2].x = 190;
// 	// warp_src_point[2].y = 170;
// 	// warp_src_point[3].x = 450;
// 	// warp_src_point[3].y = 170;
// 	warp_src_point[0].x = 5;
// 	warp_src_point[0].y = height;
// 	warp_src_point[1].x = width - warp_src_point[0].x;
// 	warp_src_point[1].y = warp_src_point[0].y;
// 	warp_src_point[2].x = 280;
// 	warp_src_point[2].y = 80;
// 	warp_src_point[3].x = width - warp_src_point[2].x;
// 	warp_src_point[3].y = warp_src_point[2].y;

// 	// warp_dst_point[0].x = 150;
// 	// warp_dst_point[0].y = height;
// 	// warp_dst_point[1].x = width - 150;
// 	// warp_dst_point[1].y = height;
// 	// warp_dst_point[2].x = 150;
// 	// warp_dst_point[2].y = 0;
// 	// warp_dst_point[3].x = width - 150;
// 	// warp_dst_point[3].y = 0;
// 	warp_dst_point[0].x = 150;
// 	warp_dst_point[0].y = height;
// 	warp_dst_point[1].x = width - warp_dst_point[0].x;
// 	warp_dst_point[1].y = height;
// 	warp_dst_point[2].x = 150;
// 	warp_dst_point[2].y = 0;
// 	warp_dst_point[3].x = width - warp_dst_point[2].x;
// 	warp_dst_point[3].y = 0;

// 	warp_matrix = cv::getPerspectiveTransform(warp_src_point, warp_dst_point);
    
//     invert(warp_matrix, warp_matrix_inv);
	
//     Mat dst;
    
//     if(num == 0)
//     {
// 	    cv::warpPerspective(img, dst, warp_matrix, cv::Size(width, height));
//     }
//     else
//     {
// 	    cv::warpPerspective(img, dst, warp_matrix_inv, cv::Size(width, height));
//     }
// 	//imshow("dst", dst); top view

// 	return dst;
// }