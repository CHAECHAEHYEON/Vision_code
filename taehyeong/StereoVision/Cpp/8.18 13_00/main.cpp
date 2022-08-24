#include "StereoVision.h"


using namespace cv;
using namespace std;

void imageCallback(const sensor_msgs::ImageConstPtr &msg);
void parking_cb_1(const std_msgs::Bool::ConstPtr &msgs);
void parking_cb_2(const std_msgs::Bool::ConstPtr &msgs);
void parking_cb_3(const std_msgs::Bool::ConstPtr &msgs);

Mat find_stopline(Mat &img);
Mat mask_filter(Mat &img, int _mask_w, int _mask_h, int thresh);
Mat bird_eyes_view_for_front_camara(Mat &img, int num);

Mat front_camera;

int isStop = 10;

float baseline = 23; //카메라 사이 거리
float focal_pixels = 800; // size(1280, 720) 일때 focal_pixels
float alpha = 20;	//alpha = 카메라 머리 숙인 각도
float beta = 45; 	//beta = erp 헤딩으로부터 카메라 각도

Mat img_color;
Mat img_color_2;
int H, S, V;
void mouse_callback(int event, int x, int y, int flags, void *param);
void mouse_callback_2(int event, int x, int y, int flags, void *param);
void on_mouse(int event, int x, int y, int flags, void *);

bool callback = false;
bool parking_level_1 = false;
bool parking_level_2 = false;
bool parking_level_3 = false;


ros::Subscriber Parking_level1_sub;
ros::Subscriber Parking_level2_sub;
ros::Subscriber Parking_level3_sub;
ros::Publisher stopline_pub;

Point ptOld1;

float camera_values();

int main(int argc, char **argv) 
{   
	ros::init(argc, argv, "parking_publisher");

    // VideoCapture capLeft(2);
    // VideoCapture capRight(4);
    VideoCapture capLeft("/home/korus/catkin_ws/src/record_video/parking/k-city/kcity_parking1.mp4");
    VideoCapture capRight("/home/korus/catkin_ws/src/record_video/parking/k-city/kcity_parking1.mp4");
    
	if (!capLeft.isOpened()) {
        cout << "Cannot Open Left Camera" << endl;
    }
    if (!capRight.isOpened()) {
        cout << "Cannot Open Right Camera" << endl;
    }

	ros::NodeHandle nh;
    ros::Publisher center_XZ_pub = nh.advertise<std_msgs::Float64MultiArray>("center_XZ", 10);
	stopline_pub = nh.advertise<std_msgs::Float64>("stopline_parking", 100);
	Parking_level1_sub = nh.subscribe("Lidar_Stop", 10, parking_cb_1); //lidar parkingstop
    Parking_level2_sub = nh.subscribe("/parking/cal", 10, parking_cb_2); //off publish coordinate
    Parking_level3_sub = nh.subscribe("/parking/cam", 10, parking_cb_3); //on stopline distance

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub_image = it.subscribe("/camera/vision/image_raw", 100, imageCallback); //subscribe front image cam

    capLeft.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    capLeft.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    capRight.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    capRight.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    StereoVision stereovision(baseline, focal_pixels, alpha, beta);

	Mat leftFrame, rightFrame;
    Mat leftMask, rightMask;
    Mat leftResFrame, rightResFrame;
    Mat front_camera_clone;
	Mat stopline_image;
	
    Point leftCircle, rightCircle; // using yellow ball
    Point2f top_XZ, mid_XZ, bottom_XZ; // using parking
        
    float left_array[6] = {0,0,0,0,0,0};
    float right_array[6] = {0,0,0,0,0,0};

    while (ros::ok) 
    {
        if(callback == false)
        {
            cout << " waiting for front camera" << endl;
        }
        else
		{
            capLeft.read(leftFrame);
            capRight.read(rightFrame);
                
            resize(leftFrame, leftFrame, Size(1280, 720));
            resize(rightFrame, rightFrame, Size(1280, 720));
            
            // stereovision.line_symmetry(leftFrame, 0); //카메라 세팅 단계에서 사용
            // stereovision.line_symmetry(rightFrame, 1);

            // Calibration of the frames (640,480)
            // leftFrame = stereovision.undistortFrame(leftFrame);
            // rightFrame = stereovision.undistortFrame(rightFrame);
            
            // rectangle(leftFrame, Rect(0, 0, 1280, 100), Scalar(0, 0, 0), -1); //상단 for koreatech
            // rectangle(rightFrame, Rect(0, 0, 1280, 100), Scalar(0, 0, 0), -1); //상단 for koreatech
            rectangle(leftFrame, Rect(0, 0, 1280, 200), Scalar(0, 0, 0), -1); //for k-citys
            rectangle(rightFrame, Rect(0, 0, 1280, 200), Scalar(0, 0, 0), -1); //for k-citys

            resize(front_camera, front_camera, Size(640, 480));
            // imshow("aaaaaa",front_camera);
            front_camera_clone = front_camera.clone();
            // Frames after applyting HSV-filter mask
            leftMask = stereovision.add_HSV_filter(leftFrame, 0);
            rightMask = stereovision.add_HSV_filter(rightFrame, 1);

            bitwise_and(leftFrame, leftFrame, leftResFrame, leftMask);
            bitwise_and(rightFrame, rightFrame, rightResFrame, rightMask);
			// imshow("leftResFrame",leftResFrame );

			// imshow("Left Frame", leftFrame);
			// imshow("Right Frame", rightFrame);

			// img_color = leftFrame.clone();
			// img_color_2 = rightFrame.clone();
			// setMouseCallback("Left Frame", mouse_callback);
			// setMouseCallback("Right Frame", mouse_callback_2);

			// setMouseCallback("Left Frame",on_mouse);
			// setMouseCallback("Right Frame",on_mouse);

            if((parking_level_1 == false) && (parking_level_2 == false) && (parking_level_3 == false))
            {
                cout << " Wating Lidar Stop" << endl;
				imshow("Left Mask", leftMask);
           		imshow("Right Mask", rightMask);
            }
            else if((parking_level_1 == true) && (parking_level_2 == false) && (parking_level_3 == false))
            {
                
                // Detect Circles - Hough Transforms can be used aswell or some neural network to do object detection
                // leftCircle = stereovision.find_ball(leftMask, leftMask);
                // rightCircle = stereovision.find_ball(rightMask, rightMask);

                float *ptr_left = stereovision.mass_center(leftMask, left_array, 0);
                float *ptr_right = stereovision.mass_center(rightMask, right_array, 1);

                // If no ball is detected in one of the cameras - show the text "tracking lost"
                if (!left_array[2] || !right_array[2])
                {
                    putText(leftMask, "Tracking Lost!", { 75, 50 }, FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2);
                    putText(rightMask, "Tracking Lost!", { 75, 75 }, FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2);
                } 
                else
                {
                    bottom_XZ = stereovision.find_XZ({left_array[0],left_array[1]}, {right_array[0],right_array[1]}, leftFrame, rightFrame);
                    mid_XZ = stereovision.find_XZ({left_array[2],left_array[3]}, {right_array[2],right_array[3]}, leftFrame, rightFrame);
                    top_XZ = stereovision.find_XZ({left_array[4],left_array[5]}, {right_array[4],right_array[5]}, leftFrame, rightFrame);

                    putText(leftMask, "Tracking!", { 75, 50 }, FONT_HERSHEY_SIMPLEX, 0.7, (50,170,50), 2);
                    putText(rightMask, "Tracking!", { 75, 75 }, FONT_HERSHEY_SIMPLEX, 0.7, (50,170,50), 2);
                }
                cout << "bottom_XZ : " << bottom_XZ << endl;
                cout << "mid_XZ : " << mid_XZ << endl;
                cout << "top_XZ : " << top_XZ << endl << endl;

                float arr[6];
                arr[0]= bottom_XZ.x/100.00;
                arr[1]= bottom_XZ.y/100.00;
                arr[2]= mid_XZ.x/100.00;
                arr[3]= mid_XZ.y/100.00;
                arr[4]= top_XZ.x/100.00;
                arr[5]= top_XZ.y/100.00;

                std_msgs::Float64MultiArray center_XZ_msg;
                center_XZ_msg.data.clear();
                for(int i=0; i<6; i++)
                {
                    center_XZ_msg.data.push_back(arr[i]);
                    // printf("arr[] : %f\n", arr[i]);
                }
                center_XZ_pub.publish(center_XZ_msg);
            }
			else if((parking_level_1 == true) && (parking_level_2 == true) && (parking_level_3 == false))
			{
				cout << " waiting for signal " << endl;
			}
            else if((parking_level_1 == true) && (parking_level_2 == true) && (parking_level_3 == true))
            {
                stopline_image = find_stopline(front_camera);
                // imshow("stopline_image",stopline_image);
                
				Mat stopline_final;
                addWeighted(front_camera_clone, 0.5, stopline_image, 0.5, 0, stopline_final);

                imshow("stopline_final", stopline_final);
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

void parking_cb_1(const std_msgs::Bool::ConstPtr &msgs)
{
	parking_level_1 = msgs->data;
}

void parking_cb_2(const std_msgs::Bool::ConstPtr &msgs)
{
	parking_level_2 = msgs->data;
}

void parking_cb_3(const std_msgs::Bool::ConstPtr &msgs)
{
	parking_level_3 = msgs->data;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	// printf("이미지 수신중\n");
	Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
	front_camera = image.clone();
	cv::waitKey(1);
	callback = true;
	// imshow("image", image);
}

Mat find_stopline(Mat &img)
{
	Mat img_clone = img.clone();
	
	Mat HSV_image;
	cvtColor(img, HSV_image, COLOR_BGR2HSV);

	Scalar lower_white = Scalar(0, 0, 170);
	Scalar upper_white = Scalar(255, 100, 255);

	Mat white_image;
	inRange(HSV_image, lower_white, upper_white, white_image);
	// imshow("white_image", white_image);

	Mat warp_image;
	warp_image = bird_eyes_view_for_front_camara(white_image, 0);
	imshow("warp_img", warp_image);

	Mat warp_image_clone = warp_image.clone();

	Mat integtal_image;
	cv::integral(warp_image_clone, integtal_image);

	Mat mask_image;
	mask_image = mask_filter(integtal_image, 5, 5, 90);
	//imshow("mask_image", mask_image);

	Mat warp_inverse_image;
	warp_inverse_image = bird_eyes_view_for_front_camara(mask_image, 1);
	// imshow("warp_inverse_image", warp_inverse_image);

	resize(warp_inverse_image, warp_inverse_image, Size(640, 480), 0, 0);
	// imshow("img_resize",warp_inverse_image);

	Mat final_image;
	addWeighted(img_clone, 0.5, warp_inverse_image, 0.5, 0, final_image);

	int count = 100;

	if (isStop == 5)
	{
		count = 5;
		putText(final_image, "5M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
	}
	else if (isStop == 4)
	{
		count = 4;
		putText(final_image, "4M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
	}
	else if (isStop == 3)
	{
		count = 3;
		putText(final_image, "3M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
	}
	else if (isStop == 2)
	{
		count = 2;
		putText(final_image, "2M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
	}
	else if (isStop == 1)
	{
		count = 1;
		putText(final_image, "1M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
	}
	else if (isStop == 10)
	{
		putText(final_image, "GO!", Point(380, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
	}

	std_msgs::Float64 stopline_msg;
	stopline_msg.data = count;
	// cout << "stopline" << endl;
	// ROS_INFO("%f", stopline_msg.data);
	stopline_pub.publish(stopline_msg);						// 메시지를발행한다

	// imshow("final_image", final_image);

	return final_image;
}

Mat mask_filter(Mat &img, int _mask_w, int _mask_h, int thresh)
{
	int height = img.rows;
	int width = img.cols;
	Mat img_maskfilter;
	img_maskfilter = Mat::zeros(height, width, CV_8UC1);
	Mat img_stop;
	img_stop = Mat::zeros(height, width, CV_8UC3);
	float mask[3];
	isStop = 10;

	// putText(img_stop, "7M", Point(0, 40), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
	// line(img_stop, Point(3, 40), Point(600, 40), Scalar(0, 0, 255));
	// putText(img_stop, "6M", Point(0, 140), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255));
	// line(img_stop, Point(3, 140), Point(600, 140), Scalar(255, 0, 255));
	// putText(img_stop, "5M", Point(0, 240), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255));
	// line(img_stop, Point(3, 240), Point(600, 240), Scalar(0, 255, 255));
	// putText(img_stop, "4M", Point(0, 340), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 0));
	// line(img_stop, Point(3, 340), Point(600, 340), Scalar(255, 255, 0));
	// putText(img_stop, "3M", Point(0, 440), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
	// line(img_stop, Point(3, 440), Point(600, 440), Scalar(0, 0, 255));

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
			if (sum > 8000)
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
				cout << "stop line distance : 5M"<< endl << endl;
				isStop = 5;
			}
			else if (y < 230)
			{
				cout << "stop line distance : 4M" << endl << endl;
				isStop = 4;
			}
			else if (y < 267)
			{
				cout << "stop line distance : 3M" << endl << endl;

				isStop = 3;
			}
			else if (y < 330)
			{
				cout << "stop line distance : 2M" << endl << endl;
				isStop = 2;
			}
			else
			{
				cout << "stop line distance : 1M" << endl << endl;
				isStop = 1;
			}
			break;
		}
	}
	// imshow("img_stop",img_stop);
	return img_stop;
}

Mat bird_eyes_view_for_front_camara(Mat &img, int num)
{
	int width = img.cols;
	int height = img.rows;

	width = img.cols;
	height = img.rows;
	Mat warp_matrix;
    Mat warp_matrix_inv;

	Point2f warp_src_point[4];
	Point2f warp_dst_point[4];

	warp_src_point[0].x = 40;
	warp_src_point[0].y = 480;
	warp_src_point[1].x = 600;
	warp_src_point[1].y = 480;
	warp_src_point[2].x = 190;
	warp_src_point[2].y = 170;
	warp_src_point[3].x = 450;
	warp_src_point[3].y = 170;

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
	
    Mat dst;
    
    if(num == 0)
    {
	    cv::warpPerspective(img, dst, warp_matrix, cv::Size(width, height));
    }
    else
    {
	    cv::warpPerspective(img, dst, warp_matrix_inv, cv::Size(width, height));
    }
	//imshow("dst", dst); top view

	return dst;
}